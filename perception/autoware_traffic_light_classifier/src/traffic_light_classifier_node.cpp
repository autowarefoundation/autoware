// Copyright 2023 TIER IV, Inc.
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
#include "traffic_light_classifier_node.hpp"

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{
TrafficLightClassifierNodelet::TrafficLightClassifierNodelet(const rclcpp::NodeOptions & options)
: Node("traffic_light_classifier_node", options)
{
  classify_traffic_light_type_ = this->declare_parameter("classify_traffic_light_type", 0);

  using std::placeholders::_1;
  using std::placeholders::_2;
  is_approximate_sync_ = this->declare_parameter("approximate_sync", false);
  backlight_threshold_ = this->declare_parameter<double>("backlight_threshold");

  if (is_approximate_sync_) {
    approximate_sync_.reset(new ApproximateSync(ApproximateSyncPolicy(10), image_sub_, roi_sub_));
    approximate_sync_->registerCallback(
      std::bind(&TrafficLightClassifierNodelet::imageRoiCallback, this, _1, _2));
  } else {
    sync_.reset(new Sync(SyncPolicy(10), image_sub_, roi_sub_));
    sync_->registerCallback(
      std::bind(&TrafficLightClassifierNodelet::imageRoiCallback, this, _1, _2));
  }

  traffic_signal_array_pub_ = this->create_publisher<tier4_perception_msgs::msg::TrafficLightArray>(
    "~/output/traffic_signals", rclcpp::QoS{1});

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&TrafficLightClassifierNodelet::connectCb, this));

  int classifier_type = this->declare_parameter(
    "classifier_type", static_cast<int>(TrafficLightClassifierNodelet::ClassifierType::HSVFilter));
  if (classifier_type == TrafficLightClassifierNodelet::ClassifierType::HSVFilter) {
    classifier_ptr_ = std::make_shared<ColorClassifier>(this);
  } else if (classifier_type == TrafficLightClassifierNodelet::ClassifierType::CNN) {
#if ENABLE_GPU
    classifier_ptr_ = std::make_shared<CNNClassifier>(this);
#else
    RCLCPP_ERROR(
      this->get_logger(), "please install CUDA, CUDNN and TensorRT to use cnn classifier");
#endif
  }
}

void TrafficLightClassifierNodelet::connectCb()
{
  // set callbacks only when there are subscribers to this node
  if (
    traffic_signal_array_pub_->get_subscription_count() == 0 &&
    traffic_signal_array_pub_->get_intra_process_subscription_count() == 0) {
    image_sub_.unsubscribe();
    roi_sub_.unsubscribe();
  } else if (!image_sub_.getSubscriber()) {
    image_sub_.subscribe(this, "~/input/image", "raw", rmw_qos_profile_sensor_data);
    roi_sub_.subscribe(this, "~/input/rois", rclcpp::QoS{1}.get_rmw_qos_profile());
  }
}

void TrafficLightClassifierNodelet::imageRoiCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
  const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_rois_msg)
{
  if (classifier_ptr_.use_count() == 0) {
    return;
  }
  tier4_perception_msgs::msg::TrafficLightArray output_msg;
  if (input_rois_msg->rois.empty()) {
    output_msg.header = input_image_msg->header;
    traffic_signal_array_pub_->publish(output_msg);
    return;
  }

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not convert from '%s' to 'rgb8'.",
      input_image_msg->encoding.c_str());
  }

  output_msg.signals.resize(input_rois_msg->rois.size());

  std::vector<cv::Mat> images;
  std::vector<size_t> backlight_indices;
  size_t idx_valid_roi = 0;
  for (const auto & input_roi : input_rois_msg->rois) {
    // ignore if the roi is not the type to be classified
    if (input_roi.traffic_light_type != classify_traffic_light_type_) {
      continue;
    }
    // skip if the roi size is zero
    if (input_roi.roi.height == 0 || input_roi.roi.width == 0) {
      continue;
    }

    // set traffic light id and type
    output_msg.signals[idx_valid_roi].traffic_light_id = input_roi.traffic_light_id;
    output_msg.signals[idx_valid_roi].traffic_light_type = input_roi.traffic_light_type;

    const sensor_msgs::msg::RegionOfInterest & roi = input_roi.roi;
    auto roi_img = cv_ptr->image(cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
    if (is_harsh_backlight(roi_img)) {
      backlight_indices.emplace_back(idx_valid_roi);
    }
    images.emplace_back(roi_img);
    idx_valid_roi++;
  }

  // classify the images
  output_msg.signals.resize(images.size());
  if (!images.empty()) {
    if (!classifier_ptr_->getTrafficSignals(images, output_msg)) {
      RCLCPP_ERROR(this->get_logger(), "failed classify image, abort callback");
      return;
    }
  }

  // append the undetected rois as unknown
  for (const auto & input_roi : input_rois_msg->rois) {
    // if the type is the target type but the roi size is zero, the roi is undetected
    if (
      (input_roi.roi.height == 0 || input_roi.roi.width == 0) &&
      input_roi.traffic_light_type == classify_traffic_light_type_) {
      tier4_perception_msgs::msg::TrafficLight tlr_sig;
      tlr_sig.traffic_light_id = input_roi.traffic_light_id;
      tlr_sig.traffic_light_type = input_roi.traffic_light_type;
      tier4_perception_msgs::msg::TrafficLightElement element;
      element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
      element.confidence = 0.0;
      tlr_sig.elements.push_back(element);
      output_msg.signals.push_back(tlr_sig);
    }
  }

  for (const auto & idx : backlight_indices) {
    auto & elements = output_msg.signals.at(idx).elements;
    for (auto & element : elements) {
      element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      element.confidence = 0.0;
    }
  }

  output_msg.header = input_image_msg->header;
  traffic_signal_array_pub_->publish(output_msg);
}

bool TrafficLightClassifierNodelet::is_harsh_backlight(const cv::Mat & img) const
{
  if (img.empty()) {
    return false;
  }
  cv::Mat y_cr_cb;
  cv::cvtColor(img, y_cr_cb, cv::COLOR_RGB2YCrCb);

  const cv::Scalar mean_values = cv::mean(y_cr_cb);
  const double intensity = (mean_values[0] - 112.5) / 112.5;

  return backlight_threshold_ <= intensity;
}

}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightClassifierNodelet)
