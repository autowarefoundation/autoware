// Copyright 2022 Tier IV, Inc.
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

#include <tensorrt_yolox/tensorrt_yolox_node.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace tensorrt_yolox
{
TrtYoloXNode::TrtYoloXNode(const rclcpp::NodeOptions & node_options)
: Node("tensorrt_yolox", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  std::string model_path = declare_parameter("model_path", "");
  std::string label_path = declare_parameter("label_path", "");
  std::string precision = declare_parameter("precision", "fp32");
  // Objects with a score lower than this value will be ignored.
  // This threshold will be ignored if specified model contains EfficientNMS_TRT module in it
  float score_threshold = declare_parameter("score_threshold", 0.3);
  // Detection results will be ignored if IoU over this value.
  // This threshold will be ignored if specified model contains EfficientNMS_TRT module in it
  float nms_threshold = declare_parameter("nms_threshold", 0.7);

  if (!readLabelFile(label_path)) {
    RCLCPP_ERROR(this->get_logger(), "Could not find label file");
    rclcpp::shutdown();
  }
  trt_yolox_ = std::make_unique<tensorrt_yolox::TrtYoloX>(
    model_path, precision, label_map_.size(), score_threshold, nms_threshold);

  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrtYoloXNode::onConnect, this));

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "~/out/objects", 1);
  image_pub_ = image_transport::create_publisher(this, "~/out/image");
}

void TrtYoloXNode::onConnect()
{
  using std::placeholders::_1;
  if (
    objects_pub_->get_subscription_count() == 0 &&
    objects_pub_->get_intra_process_subscription_count() == 0 &&
    image_pub_.getNumSubscribers() == 0) {
    image_sub_.shutdown();
  } else if (!image_sub_) {
    image_sub_ = image_transport::create_subscription(
      this, "~/in/image", std::bind(&TrtYoloXNode::onImage, this, _1), "raw",
      rmw_qos_profile_sensor_data);
  }
}

void TrtYoloXNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  const auto width = in_image_ptr->image.cols;
  const auto height = in_image_ptr->image.rows;

  tensorrt_yolox::ObjectArrays objects;
  if (!trt_yolox_->doInference({in_image_ptr->image}, objects)) {
    RCLCPP_WARN(this->get_logger(), "Fail to inference");
    return;
  }
  for (const auto & yolox_object : objects.at(0)) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = yolox_object.x_offset;
    object.feature.roi.y_offset = yolox_object.y_offset;
    object.feature.roi.width = yolox_object.width;
    object.feature.roi.height = yolox_object.height;
    object.object.classification.emplace_back(autoware_auto_perception_msgs::build<Label>()
                                                .label(Label::UNKNOWN)
                                                .probability(yolox_object.score));
    if (label_map_[yolox_object.type] == "CAR") {
      object.object.classification.front().label = Label::CAR;
    } else if (
      label_map_[yolox_object.type] == "PEDESTRIAN" || label_map_[yolox_object.type] == "PERSON") {
      object.object.classification.front().label = Label::PEDESTRIAN;
    } else if (label_map_[yolox_object.type] == "BUS") {
      object.object.classification.front().label = Label::BUS;
    } else if (label_map_[yolox_object.type] == "TRUCK") {
      object.object.classification.front().label = Label::TRUCK;
    } else if (label_map_[yolox_object.type] == "BICYCLE") {
      object.object.classification.front().label = Label::BICYCLE;
    } else if (label_map_[yolox_object.type] == "MOTORCYCLE") {
      object.object.classification.front().label = Label::MOTORCYCLE;
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

  out_objects.header = msg->header;
  objects_pub_->publish(out_objects);
}

bool TrtYoloXNode::readLabelFile(const std::string & label_path)
{
  std::ifstream label_file(label_path);
  if (!label_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open label file. [%s]", label_path.c_str());
    return false;
  }
  int label_index{};
  std::string label;
  while (getline(label_file, label)) {
    std::transform(
      label.begin(), label.end(), label.begin(), [](auto c) { return std::toupper(c); });
    label_map_.insert({label_index, label});
    ++label_index;
  }
  return true;
}

}  // namespace tensorrt_yolox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tensorrt_yolox::TrtYoloXNode)
