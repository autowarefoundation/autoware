// Copyright 2020 TIER IV, Inc.
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

#include "traffic_light_ssd_fine_detector/nodelet.hpp"

#include <cuda_utils.hpp>

#if (defined(_MSC_VER) or (defined(__GNUC__) and (7 <= __GNUC_MAJOR__)))
#include <filesystem>
namespace fs = ::std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = ::std::experimental::filesystem;
#endif

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace traffic_light
{
inline std::vector<float> toFloatVector(const std::vector<double> double_vector)
{
  return std::vector<float>(double_vector.begin(), double_vector.end());
}

TrafficLightSSDFineDetectorNodelet::TrafficLightSSDFineDetectorNodelet(
  const rclcpp::NodeOptions & options)
: Node("traffic_light_ssd_fine_detector_node", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  std::vector<std::string> labels;
  const int max_batch_size = this->declare_parameter("max_batch_size", 8);
  const std::string onnx_file = this->declare_parameter<std::string>("onnx_file");
  const std::string label_file = this->declare_parameter<std::string>("label_file");
  const std::string mode = this->declare_parameter("mode", "FP32");
  dnn_header_type_ = this->declare_parameter("dnn_header_type", "pytorch");

  if (dnn_header_type_.compare("pytorch") != 0 && dnn_header_type_.compare("mmdetection") != 0) {
    RCLCPP_ERROR(
      this->get_logger(), "Unexpected DNN type: %s, choose from (pytorch, mmdetection)",
      dnn_header_type_.c_str());
  }

  fs::path engine_path{onnx_file};
  engine_path.replace_extension("engine");

  if (readLabelFile(label_file, labels)) {
    if (!getTlrIdFromLabel(labels, tlr_id_)) {
      RCLCPP_ERROR(this->get_logger(), "Could not find tlr id");
    }
  }

  if (fs::exists(engine_path)) {
    RCLCPP_INFO(this->get_logger(), "Found %s", engine_path.string().c_str());
    net_ptr_.reset(new ssd::Net(engine_path, false));
    if (max_batch_size != net_ptr_->getMaxBatchSize()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Required max batch size %d does not correspond to Profile max batch size %d. Rebuild "
        "engine "
        "from onnx",
        max_batch_size, net_ptr_->getMaxBatchSize());
      net_ptr_.reset(new ssd::Net(onnx_file, mode, max_batch_size));
      net_ptr_->save(engine_path);
    }
  } else {
    RCLCPP_INFO(
      this->get_logger(), "Could not find %s, try making TensorRT engine from onnx",
      engine_path.string().c_str());
    net_ptr_.reset(new ssd::Net(onnx_file, mode, max_batch_size));
    net_ptr_->save(engine_path);
  }

  input_shape_ = net_ptr_->getInputShape();
  box_dims_ = net_ptr_->getOutputDimensions("boxes");
  score_dims_ = net_ptr_->getOutputDimensions("scores");
  detection_per_class_ = score_dims_->d[0];
  class_num_ = score_dims_->d[1];

  is_approximate_sync_ = this->declare_parameter<bool>("approximate_sync", false);
  score_thresh_ = this->declare_parameter<double>("score_thresh", 0.7);
  mean_ = toFloatVector(this->declare_parameter("mean", std::vector<double>({0.5, 0.5, 0.5})));
  std_ = toFloatVector(this->declare_parameter("std", std::vector<double>({0.5, 0.5, 0.5})));

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficLightSSDFineDetectorNodelet::connectCb, this));

  std::lock_guard<std::mutex> lock(connect_mutex_);
  output_roi_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>("~/output/rois", 1);
  exe_time_pub_ =
    this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/exe_time_ms", 1);
  if (is_approximate_sync_) {
    approximate_sync_.reset(new ApproximateSync(ApproximateSyncPolicy(10), image_sub_, roi_sub_));
    approximate_sync_->registerCallback(
      std::bind(&TrafficLightSSDFineDetectorNodelet::callback, this, _1, _2));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_));
    sync_->registerCallback(std::bind(&TrafficLightSSDFineDetectorNodelet::callback, this, _1, _2));
  }

  if (this->declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
}

void TrafficLightSSDFineDetectorNodelet::connectCb()
{
  std::lock_guard<std::mutex> lock(connect_mutex_);
  if (output_roi_pub_->get_subscription_count() == 0) {
    image_sub_.unsubscribe();
    roi_sub_.unsubscribe();
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  }
}

void TrafficLightSSDFineDetectorNodelet::callback(
  const sensor_msgs::msg::Image::ConstSharedPtr in_image_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg)
{
  if (in_image_msg->width < 2 || in_image_msg->height < 2) {
    return;
  }

  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;
  const auto exe_start_time = high_resolution_clock::now();
  cv::Mat original_image;
  tier4_perception_msgs::msg::TrafficLightRoiArray out_rois;

  rosMsg2CvMat(in_image_msg, original_image);
  int num_rois = in_roi_msg->rois.size();
  int batch_count = 0;
  const int batch_size = net_ptr_->getMaxBatchSize();
  while (num_rois != 0) {
    const int num_infer = (num_rois / batch_size > 0) ? batch_size : num_rois % batch_size;
    auto data_d = cuda::make_unique<float[]>(num_infer * input_shape_.size());
    auto box_d = cuda::make_unique<float[]>(num_infer * box_dims_->size());
    auto score_d = cuda::make_unique<float[]>(num_infer * score_dims_->size());
    std::vector<void *> buffers;
    if (dnn_header_type_.compare("pytorch") == 0) {
      buffers = {data_d.get(), score_d.get(), box_d.get()};
    } else {
      buffers = {data_d.get(), box_d.get(), score_d.get()};
    }
    std::vector<cv::Point> lts, rbs;
    std::vector<cv::Mat> cropped_imgs;

    for (int i = 0; i < num_infer; ++i) {
      int roi_index = i + batch_count * batch_size;
      lts.push_back(cv::Point(
        in_roi_msg->rois.at(roi_index).roi.x_offset, in_roi_msg->rois.at(roi_index).roi.y_offset));
      rbs.push_back(cv::Point(
        in_roi_msg->rois.at(roi_index).roi.x_offset + in_roi_msg->rois.at(roi_index).roi.width,
        in_roi_msg->rois.at(roi_index).roi.y_offset + in_roi_msg->rois.at(roi_index).roi.height));
      fitInFrame(lts.at(i), rbs.at(i), cv::Size(original_image.size()));
      cropped_imgs.push_back(cv::Mat(original_image, cv::Rect(lts.at(i), rbs.at(i))));
    }

    std::vector<float> data(num_infer * input_shape_.size());
    if (!cvMat2CnnInput(cropped_imgs, num_infer, data)) {
      RCLCPP_ERROR(this->get_logger(), "Fail to preprocess image");
      return;
    }

    cudaMemcpy(data_d.get(), data.data(), data.size() * sizeof(float), cudaMemcpyHostToDevice);

    try {
      net_ptr_->infer(buffers, num_infer);
    } catch (std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      return;
    }

    auto scores = std::make_unique<float[]>(num_infer * detection_per_class_ * class_num_);
    auto boxes = std::make_unique<float[]>(num_infer * detection_per_class_ * 4);
    cudaMemcpy(
      boxes.get(), box_d.get(), sizeof(float) * num_infer * box_dims_->size(),
      cudaMemcpyDeviceToHost);
    cudaMemcpy(
      scores.get(), score_d.get(), sizeof(float) * num_infer * score_dims_->size(),
      cudaMemcpyDeviceToHost);
    // Get Output
    std::vector<Detection> detections;
    if (!cnnOutput2BoxDetection(
          scores.get(), boxes.get(), tlr_id_, cropped_imgs, num_infer, detections)) {
      RCLCPP_ERROR(this->get_logger(), "Fail to postprocess image");
      return;
    }

    for (int i = 0; i < num_infer; ++i) {
      if (detections.at(i).prob > score_thresh_) {
        cv::Point lt_roi =
          cv::Point(lts.at(i).x + detections.at(i).x, lts.at(i).y + detections.at(i).y);
        cv::Point rb_roi = cv::Point(
          lts.at(i).x + detections.at(i).x + detections.at(i).w,
          lts.at(i).y + detections.at(i).y + detections.at(i).h);
        fitInFrame(lt_roi, rb_roi, cv::Size(original_image.size()));
        tier4_perception_msgs::msg::TrafficLightRoi tl_roi;
        cvRect2TlRoiMsg(
          cv::Rect(lt_roi, rb_roi),
          in_roi_msg->rois.at(i + batch_count * batch_size).traffic_light_id, tl_roi);
        out_rois.rois.push_back(tl_roi);
      }
    }
    num_rois -= num_infer;
    ++batch_count;
  }
  out_rois.header = in_roi_msg->header;
  output_roi_pub_->publish(out_rois);
  const auto exe_end_time = high_resolution_clock::now();
  const double exe_time =
    std::chrono::duration_cast<milliseconds>(exe_end_time - exe_start_time).count();
  tier4_debug_msgs::msg::Float32Stamped exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_msg.stamp = this->now();
  exe_time_pub_->publish(exe_time_msg);
}

bool TrafficLightSSDFineDetectorNodelet::cvMat2CnnInput(
  const std::vector<cv::Mat> & in_imgs, const int num_rois, std::vector<float> & data)
{
  for (int i = 0; i < num_rois; ++i) {
    // cv::Mat rgb;
    // cv::cvtColor(in_imgs.at(i), rgb, CV_BGR2RGB);
    cv::Mat resized;
    cv::resize(in_imgs.at(i), resized, cv::Size(input_shape_.width, input_shape_.height));

    cv::Mat pixels;
    resized.convertTo(pixels, CV_32FC3, 1.0 / 255, 0);
    std::vector<float> img;
    if (pixels.isContinuous()) {
      img.assign(
        reinterpret_cast<const float *>(pixels.datastart),
        reinterpret_cast<const float *>(pixels.dataend));
    } else {
      return false;
    }

    for (int c = 0; c < input_shape_.channel; ++c) {
      for (int j = 0, hw = input_shape_.area(); j < hw; ++j) {
        data[i * input_shape_.size() + c * hw + j] =
          (img[input_shape_.channel * j + 2 - c] - mean_[c]) / std_[c];
      }
    }
  }
  return true;
}

bool TrafficLightSSDFineDetectorNodelet::cnnOutput2BoxDetection(
  const float * scores, const float * boxes, const int tlr_id, const std::vector<cv::Mat> & in_imgs,
  const int num_rois, std::vector<Detection> & detections)
{
  if (tlr_id > class_num_ - 1) {
    return false;
  }
  for (int i = 0; i < num_rois; ++i) {
    std::vector<float> tlr_scores;
    Detection det;
    for (int j = 0; j < detection_per_class_; ++j) {
      tlr_scores.push_back(scores[i * detection_per_class_ * class_num_ + tlr_id + j * class_num_]);
    }
    std::vector<float>::iterator iter = std::max_element(tlr_scores.begin(), tlr_scores.end());
    size_t index = std::distance(tlr_scores.begin(), iter);
    size_t box_index = i * detection_per_class_ * 4 + index * 4;
    cv::Point lt, rb;
    if (dnn_header_type_.compare("pytorch") == 0) {
      lt.x = boxes[box_index] * in_imgs.at(i).cols;
      lt.y = boxes[box_index + 1] * in_imgs.at(i).rows;
      rb.x = boxes[box_index + 2] * in_imgs.at(i).cols;
      rb.y = boxes[box_index + 3] * in_imgs.at(i).rows;
    } else {
      const float dx = static_cast<float>(in_imgs.at(i).cols) / input_shape_.width;
      const float dy = static_cast<float>(in_imgs.at(i).rows) / input_shape_.height;
      lt.x = boxes[box_index] * dx;
      lt.y = boxes[box_index + 1] * dy;
      rb.x = boxes[box_index + 2] * dx;
      rb.y = boxes[box_index + 3] * dy;
    }
    fitInFrame(lt, rb, cv::Size(in_imgs.at(i).cols, in_imgs.at(i).rows));
    det.x = lt.x;
    det.y = lt.y;
    det.w = rb.x - lt.x;
    det.h = rb.y - lt.y;

    det.prob = tlr_scores[index];
    detections.push_back(det);
  }
  return true;
}

bool TrafficLightSSDFineDetectorNodelet::rosMsg2CvMat(
  const sensor_msgs::msg::Image::ConstSharedPtr image_msg, cv::Mat & image)
{
  try {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "rgb8");
    image = cv_image->image;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to convert sensor_msgs::msg::Image to cv::Mat \n%s", e.what());
    return false;
  }

  return true;
}

bool TrafficLightSSDFineDetectorNodelet::fitInFrame(
  cv::Point & lt, cv::Point & rb, const cv::Size & size)
{
  const int width = static_cast<int>(size.width);
  const int height = static_cast<int>(size.height);
  {
    const int x_min = 0, x_max = width - 2;
    const int y_min = 0, y_max = height - 2;
    lt.x = std::min(std::max(lt.x, x_min), x_max);
    lt.y = std::min(std::max(lt.y, y_min), y_max);
  }
  {
    const int x_min = lt.x + 1, x_max = width - 1;
    const int y_min = lt.y + 1, y_max = height - 1;
    rb.x = std::min(std::max(rb.x, x_min), x_max);
    rb.y = std::min(std::max(rb.y, y_min), y_max);
  }

  return true;
}

void TrafficLightSSDFineDetectorNodelet::cvRect2TlRoiMsg(
  const cv::Rect & rect, const int32_t id, tier4_perception_msgs::msg::TrafficLightRoi & tl_roi)
{
  tl_roi.traffic_light_id = id;
  tl_roi.roi.x_offset = rect.x;
  tl_roi.roi.y_offset = rect.y;
  tl_roi.roi.width = rect.width;
  tl_roi.roi.height = rect.height;
}

bool TrafficLightSSDFineDetectorNodelet::readLabelFile(
  std::string filepath, std::vector<std::string> & labels)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  while (getline(labelsFile, label)) {
    labels.push_back(label);
  }
  return true;
}

bool TrafficLightSSDFineDetectorNodelet::getTlrIdFromLabel(
  const std::vector<std::string> & labels, int & tlr_id)
{
  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels.at(i) == "traffic_light") {
      tlr_id = i;
      return true;
    }
  }
  return false;
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::TrafficLightSSDFineDetectorNodelet)
