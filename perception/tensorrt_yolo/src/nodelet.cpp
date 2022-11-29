// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tensorrt_yolo/nodelet.hpp"

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <glob.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
std::vector<std::string> getFilePath(const std::string & input_dir)
{
  glob_t globbuf;
  std::vector<std::string> files;
  glob((input_dir + "*").c_str(), 0, NULL, &globbuf);
  for (size_t i = 0; i < globbuf.gl_pathc; i++) {
    files.push_back(globbuf.gl_pathv[i]);
  }
  globfree(&globbuf);
  return files;
}
}  // namespace
namespace object_recognition
{
TensorrtYoloNodelet::TensorrtYoloNodelet(const rclcpp::NodeOptions & options)
: Node("tensorrt_yolo", options)
{
  using std::placeholders::_1;

  std::string onnx_file = declare_parameter("onnx_file", "");
  std::string engine_file = declare_parameter("engine_file", "");
  std::string label_file = declare_parameter("label_file", "");
  std::string calib_image_directory = declare_parameter("calib_image_directory", "");
  std::string calib_cache_file = declare_parameter("calib_cache_file", "");
  std::string mode = declare_parameter("mode", "FP32");
  gpu_device_id_ = declare_parameter("gpu_id", 0);
  yolo_config_.num_anchors = declare_parameter("num_anchors", 3);
  auto anchors = declare_parameter(
    "anchors", std::vector<double>{
                 10, 13, 16, 30, 33, 23, 30, 61, 62, 45, 59, 119, 116, 90, 156, 198, 373, 326});
  std::vector<float> anchors_float(anchors.begin(), anchors.end());
  yolo_config_.anchors = anchors_float;
  auto scale_x_y = declare_parameter("scale_x_y", std::vector<double>{1.0, 1.0, 1.0});
  std::vector<float> scale_x_y_float(scale_x_y.begin(), scale_x_y.end());
  yolo_config_.scale_x_y = scale_x_y_float;
  yolo_config_.score_thresh = declare_parameter("score_thresh", 0.1);
  yolo_config_.iou_thresh = declare_parameter("iou_thresh", 0.45);
  yolo_config_.detections_per_im = declare_parameter("detections_per_im", 100);
  yolo_config_.use_darknet_layer = declare_parameter("use_darknet_layer", true);
  yolo_config_.ignore_thresh = declare_parameter("ignore_thresh", 0.5);

  if (!yolo::set_cuda_device(gpu_device_id_)) {
    RCLCPP_ERROR(this->get_logger(), "Given GPU not exist or suitable");
  }

  if (!readLabelFile(label_file, &labels_)) {
    RCLCPP_ERROR(this->get_logger(), "Could not find label file");
  }
  std::ifstream fs(engine_file);
  const auto calibration_images = getFilePath(calib_image_directory);
  if (fs.is_open()) {
    RCLCPP_INFO(this->get_logger(), "Found %s", engine_file.c_str());
    net_ptr_.reset(new yolo::Net(engine_file, false));
    if (net_ptr_->getMaxBatchSize() != 1) {
      RCLCPP_INFO(
        this->get_logger(), "Max batch size %d should be 1. Rebuild engine from file",
        net_ptr_->getMaxBatchSize());
      net_ptr_.reset(
        new yolo::Net(onnx_file, mode, 1, yolo_config_, calibration_images, calib_cache_file));
      net_ptr_->save(engine_file);
    }
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Could not find %s, try making TensorRT engine from onnx",
      engine_file.c_str());
    net_ptr_.reset(
      new yolo::Net(onnx_file, mode, 1, yolo_config_, calibration_images, calib_cache_file));
    net_ptr_->save(engine_file);
  }
  RCLCPP_INFO(this->get_logger(), "Inference engine prepared.");

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TensorrtYoloNodelet::connectCb, this));

  std::lock_guard<std::mutex> lock(connect_mutex_);

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "out/objects", 1);
  image_pub_ = image_transport::create_publisher(this, "out/image");

  out_scores_ =
    std::make_unique<float[]>(net_ptr_->getMaxBatchSize() * net_ptr_->getMaxDetections());
  out_boxes_ =
    std::make_unique<float[]>(net_ptr_->getMaxBatchSize() * net_ptr_->getMaxDetections() * 4);
  out_classes_ =
    std::make_unique<float[]>(net_ptr_->getMaxBatchSize() * net_ptr_->getMaxDetections());
}

void TensorrtYoloNodelet::connectCb()
{
  using std::placeholders::_1;
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (objects_pub_->get_subscription_count() == 0 && image_pub_.getNumSubscribers() == 0) {
    image_sub_.shutdown();
  } else if (!image_sub_) {
    image_sub_ = image_transport::create_subscription(
      this, "in/image", std::bind(&TensorrtYoloNodelet::callback, this, _1), "raw",
      rmw_qos_profile_sensor_data);
  }
}

void TensorrtYoloNodelet::callback(const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;

  if (!yolo::set_cuda_device(gpu_device_id_)) {
    RCLCPP_ERROR(this->get_logger(), "Given GPU not exist or suitable");
    return;
  }

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  if (!net_ptr_->detect(
        in_image_ptr->image, out_scores_.get(), out_boxes_.get(), out_classes_.get())) {
    RCLCPP_WARN(this->get_logger(), "Fail to inference");
    return;
  }
  const auto width = in_image_ptr->image.cols;
  const auto height = in_image_ptr->image.rows;
  for (int i = 0; i < yolo_config_.detections_per_im; ++i) {
    if (out_scores_[i] < yolo_config_.ignore_thresh) {
      break;
    }
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = out_boxes_[4 * i] * width;
    object.feature.roi.y_offset = out_boxes_[4 * i + 1] * height;
    object.feature.roi.width = out_boxes_[4 * i + 2] * width;
    object.feature.roi.height = out_boxes_[4 * i + 3] * height;
    object.object.classification.emplace_back(autoware_auto_perception_msgs::build<Label>()
                                                .label(Label::UNKNOWN)
                                                .probability(out_scores_[i]));
    const auto class_id = static_cast<int>(out_classes_[i]);
    if (labels_[class_id] == "car") {
      object.object.classification.front().label = Label::CAR;
    } else if (labels_[class_id] == "person") {
      object.object.classification.front().label = Label::PEDESTRIAN;
    } else if (labels_[class_id] == "bus") {
      object.object.classification.front().label = Label::BUS;
    } else if (labels_[class_id] == "truck") {
      object.object.classification.front().label = Label::TRUCK;
    } else if (labels_[class_id] == "bicycle") {
      object.object.classification.front().label = Label::BICYCLE;
    } else if (labels_[class_id] == "motorbike") {
      object.object.classification.front().label = Label::MOTORCYCLE;
    } else {
      object.object.classification.front().label = Label::UNKNOWN;
    }
    out_objects.feature_objects.push_back(object);
    const auto left = std::max(0, static_cast<int>(object.feature.roi.x_offset));
    const auto top = std::max(0, static_cast<int>(object.feature.roi.y_offset));
    const auto right =
      std::min(static_cast<int>(object.feature.roi.x_offset + object.feature.roi.width), width);
    const auto bottom =
      std::min(static_cast<int>(object.feature.roi.y_offset + object.feature.roi.height), height);
    cv::rectangle(
      in_image_ptr->image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3,
      8, 0);
  }
  image_pub_.publish(in_image_ptr->toImageMsg());

  out_objects.header = in_image_msg->header;
  objects_pub_->publish(out_objects);
}

bool TensorrtYoloNodelet::readLabelFile(
  const std::string & filepath, std::vector<std::string> * labels)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  while (getline(labelsFile, label)) {
    labels->push_back(label);
  }
  return true;
}

}  // namespace object_recognition

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(object_recognition::TensorrtYoloNodelet)
