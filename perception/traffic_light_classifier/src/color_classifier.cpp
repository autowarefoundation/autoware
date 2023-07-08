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
#include "traffic_light_classifier/color_classifier.hpp"

#include <opencv2/imgproc/imgproc_c.h>

#include <algorithm>
#include <string>
#include <vector>

namespace traffic_light
{
ColorClassifier::ColorClassifier(rclcpp::Node * node_ptr) : node_ptr_(node_ptr)
{
  using std::placeholders::_1;
  image_pub_ = image_transport::create_publisher(
    node_ptr_, "~/debug/image", rclcpp::QoS{1}.get_rmw_qos_profile());

  hsv_config_.green_min_h = node_ptr_->declare_parameter("green_min_h", 50);
  hsv_config_.green_min_s = node_ptr_->declare_parameter("green_min_s", 100);
  hsv_config_.green_min_v = node_ptr_->declare_parameter("green_min_v", 150);
  hsv_config_.green_max_h = node_ptr_->declare_parameter("green_max_h", 120);
  hsv_config_.green_max_s = node_ptr_->declare_parameter("green_max_s", 200);
  hsv_config_.green_max_v = node_ptr_->declare_parameter("green_max_v", 255);
  hsv_config_.yellow_min_h = node_ptr_->declare_parameter("yellow_min_h", 0);
  hsv_config_.yellow_min_s = node_ptr_->declare_parameter("yellow_min_s", 80);
  hsv_config_.yellow_min_v = node_ptr_->declare_parameter("yellow_min_v", 150);
  hsv_config_.yellow_max_h = node_ptr_->declare_parameter("yellow_max_h", 50);
  hsv_config_.yellow_max_s = node_ptr_->declare_parameter("yellow_max_s", 200);
  hsv_config_.yellow_max_v = node_ptr_->declare_parameter("yellow_max_v", 255);
  hsv_config_.red_min_h = node_ptr_->declare_parameter("red_min_h", 160);
  hsv_config_.red_min_s = node_ptr_->declare_parameter("red_min_s", 100);
  hsv_config_.red_min_v = node_ptr_->declare_parameter("red_min_v", 150);
  hsv_config_.red_max_h = node_ptr_->declare_parameter("red_max_h", 180);
  hsv_config_.red_max_s = node_ptr_->declare_parameter("red_max_s", 255);
  hsv_config_.red_max_v = node_ptr_->declare_parameter("red_max_v", 255);

  // set parameter callback
  set_param_res_ = node_ptr_->add_on_set_parameters_callback(
    std::bind(&ColorClassifier::parametersCallback, this, _1));
}

bool ColorClassifier::getTrafficSignals(
  const std::vector<cv::Mat> & images,
  tier4_perception_msgs::msg::TrafficSignalArray & traffic_signals)
{
  if (images.size() != traffic_signals.signals.size()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "image number should be equal to traffic signal number!");
    return false;
  }
  for (size_t image_i = 0; image_i < images.size(); image_i++) {
    const auto & input_image = images[image_i];
    auto & traffic_signal = traffic_signals.signals[image_i];
    cv::Mat green_image;
    cv::Mat yellow_image;
    cv::Mat red_image;
    filterHSV(input_image, green_image, yellow_image, red_image);
    // binarize
    cv::Mat green_bin_image;
    cv::Mat yellow_bin_image;
    cv::Mat red_bin_image;
    const int bin_threshold = 127;
    cv::threshold(green_image, green_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(yellow_image, yellow_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(red_image, red_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
    // filter noise
    cv::Mat green_filtered_bin_image;
    cv::Mat yellow_filtered_bin_image;
    cv::Mat red_filtered_bin_image;
    cv::Mat element4 = (cv::Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
    cv::erode(green_bin_image, green_filtered_bin_image, element4, cv::Point(-1, -1), 1);
    cv::erode(yellow_bin_image, yellow_filtered_bin_image, element4, cv::Point(-1, -1), 1);
    cv::erode(red_bin_image, red_filtered_bin_image, element4, cv::Point(-1, -1), 1);
    cv::dilate(green_filtered_bin_image, green_filtered_bin_image, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(
      yellow_filtered_bin_image, yellow_filtered_bin_image, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(red_filtered_bin_image, red_filtered_bin_image, cv::Mat(), cv::Point(-1, -1), 1);

    /* debug */
#if 1
    if (0 < image_pub_.getNumSubscribers()) {
      cv::Mat debug_raw_image;
      cv::Mat debug_green_image;
      cv::Mat debug_yellow_image;
      cv::Mat debug_red_image;
      cv::hconcat(input_image, input_image, debug_raw_image);
      cv::hconcat(debug_raw_image, input_image, debug_raw_image);
      cv::hconcat(green_image, green_bin_image, debug_green_image);
      cv::hconcat(debug_green_image, green_filtered_bin_image, debug_green_image);
      cv::hconcat(yellow_image, yellow_bin_image, debug_yellow_image);
      cv::hconcat(debug_yellow_image, yellow_filtered_bin_image, debug_yellow_image);
      cv::hconcat(red_image, red_bin_image, debug_red_image);
      cv::hconcat(debug_red_image, red_filtered_bin_image, debug_red_image);

      cv::Mat debug_image;
      cv::vconcat(debug_green_image, debug_yellow_image, debug_image);
      cv::vconcat(debug_image, debug_red_image, debug_image);
      cv::cvtColor(debug_image, debug_image, cv::COLOR_GRAY2RGB);
      cv::vconcat(debug_raw_image, debug_image, debug_image);
      const int width = input_image.cols;
      const int height = input_image.rows;
      cv::line(
        debug_image, cv::Point(0, 0), cv::Point(debug_image.cols, 0), cv::Scalar(255, 255, 255), 1,
        CV_AA, 0);
      cv::line(
        debug_image, cv::Point(0, height), cv::Point(debug_image.cols, height),
        cv::Scalar(255, 255, 255), 1, CV_AA, 0);
      cv::line(
        debug_image, cv::Point(0, height * 2), cv::Point(debug_image.cols, height * 2),
        cv::Scalar(255, 255, 255), 1, CV_AA, 0);
      cv::line(
        debug_image, cv::Point(0, height * 3), cv::Point(debug_image.cols, height * 3),
        cv::Scalar(255, 255, 255), 1, CV_AA, 0);

      cv::line(
        debug_image, cv::Point(0, 0), cv::Point(0, debug_image.rows), cv::Scalar(255, 255, 255), 1,
        CV_AA, 0);
      cv::line(
        debug_image, cv::Point(width, 0), cv::Point(width, debug_image.rows),
        cv::Scalar(255, 255, 255), 1, CV_AA, 0);
      cv::line(
        debug_image, cv::Point(width * 2, 0), cv::Point(width * 2, debug_image.rows),
        cv::Scalar(255, 255, 255), 1, CV_AA, 0);
      cv::line(
        debug_image, cv::Point(width * 3, 0), cv::Point(width * 3, debug_image.rows),
        cv::Scalar(255, 255, 255), 1, CV_AA, 0);

      cv::putText(
        debug_image, "green", cv::Point(0, height * 1.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
        cv::Scalar(255, 255, 255), 1, CV_AA);
      cv::putText(
        debug_image, "yellow", cv::Point(0, height * 2.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
        cv::Scalar(255, 255, 255), 1, CV_AA);
      cv::putText(
        debug_image, "red", cv::Point(0, height * 3.5), cv::FONT_HERSHEY_SIMPLEX, 1.0,
        cv::Scalar(255, 255, 255), 1, CV_AA);
      const auto debug_image_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_image).toImageMsg();
      image_pub_.publish(debug_image_msg);
    }
#endif
    /* --- */

    const int green_pixel_num = cv::countNonZero(green_filtered_bin_image);
    const int yellow_pixel_num = cv::countNonZero(yellow_filtered_bin_image);
    const int red_pixel_num = cv::countNonZero(red_filtered_bin_image);
    const double green_ratio =
      static_cast<double>(green_pixel_num) /
      static_cast<double>(green_filtered_bin_image.rows * green_filtered_bin_image.cols);
    const double yellow_ratio =
      static_cast<double>(yellow_pixel_num) /
      static_cast<double>(yellow_filtered_bin_image.rows * yellow_filtered_bin_image.cols);
    const double red_ratio =
      static_cast<double>(red_pixel_num) /
      static_cast<double>(red_filtered_bin_image.rows * red_filtered_bin_image.cols);

    if (yellow_ratio < green_ratio && red_ratio < green_ratio) {
      tier4_perception_msgs::msg::TrafficLightElement element;
      element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
      element.confidence = std::min(1.0, static_cast<double>(green_pixel_num) / (20.0 * 20.0));
      traffic_signal.elements.push_back(element);
    } else if (green_ratio < yellow_ratio && red_ratio < yellow_ratio) {
      tier4_perception_msgs::msg::TrafficLightElement element;
      element.color = tier4_perception_msgs::msg::TrafficLightElement::AMBER;
      element.confidence = std::min(1.0, static_cast<double>(yellow_pixel_num) / (20.0 * 20.0));
      traffic_signal.elements.push_back(element);
    } else if (green_ratio < red_ratio && yellow_ratio < red_ratio) {
      tier4_perception_msgs::msg::TrafficLightElement element;
      element.color = ::tier4_perception_msgs::msg::TrafficLightElement::RED;
      element.confidence = std::min(1.0, static_cast<double>(red_pixel_num) / (20.0 * 20.0));
      traffic_signal.elements.push_back(element);
    } else {
      tier4_perception_msgs::msg::TrafficLightElement element;
      element.color = ::tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      element.confidence = 0.0;
      traffic_signal.elements.push_back(element);
    }
  }
  return true;
}

bool ColorClassifier::filterHSV(
  const cv::Mat & input_image, cv::Mat & green_image, cv::Mat & yellow_image, cv::Mat & red_image)
{
  cv::Mat hsv_image;
  cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
  try {
    cv::inRange(hsv_image, min_hsv_green_, max_hsv_green_, green_image);
    cv::inRange(hsv_image, min_hsv_yellow_, max_hsv_yellow_, yellow_image);
    cv::inRange(hsv_image, min_hsv_red_, max_hsv_red_, red_image);
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "failed to filter image by hsv value : %s", e.what());
    return false;
  }
  return true;
}
rcl_interfaces::msg::SetParametersResult ColorClassifier::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto update_param = [&](const std::string & name, int & v) {
    auto it = std::find_if(
      parameters.cbegin(), parameters.cend(),
      [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
    if (it != parameters.cend()) {
      v = it->as_int();
      return true;
    }
    return false;
  };

  update_param("green_min_h", hsv_config_.green_min_h);
  update_param("green_min_s", hsv_config_.green_min_s);
  update_param("green_min_v", hsv_config_.green_min_v);
  update_param("green_max_h", hsv_config_.green_max_h);
  update_param("green_max_s", hsv_config_.green_max_s);
  update_param("green_max_v", hsv_config_.green_max_v);
  update_param("yellow_min_h", hsv_config_.yellow_min_h);
  update_param("yellow_min_s", hsv_config_.yellow_min_s);
  update_param("yellow_min_v", hsv_config_.yellow_min_v);
  update_param("yellow_max_h", hsv_config_.yellow_max_h);
  update_param("yellow_max_s", hsv_config_.yellow_max_s);
  update_param("yellow_max_v", hsv_config_.yellow_max_v);
  update_param("red_min_h", hsv_config_.red_min_h);
  update_param("red_min_s", hsv_config_.red_min_s);
  update_param("red_min_v", hsv_config_.red_min_v);
  update_param("red_max_h", hsv_config_.red_max_h);
  update_param("red_max_s", hsv_config_.red_max_s);
  update_param("red_max_v", hsv_config_.red_max_v);

  min_hsv_green_ =
    cv::Scalar(hsv_config_.green_min_h, hsv_config_.green_min_s, hsv_config_.green_min_v);
  max_hsv_green_ =
    cv::Scalar(hsv_config_.green_max_h, hsv_config_.green_max_s, hsv_config_.green_max_v);
  min_hsv_yellow_ =
    cv::Scalar(hsv_config_.yellow_min_h, hsv_config_.yellow_min_s, hsv_config_.yellow_min_v);
  max_hsv_yellow_ =
    cv::Scalar(hsv_config_.yellow_max_h, hsv_config_.yellow_max_s, hsv_config_.yellow_max_v);
  min_hsv_red_ = cv::Scalar(hsv_config_.red_min_h, hsv_config_.red_min_s, hsv_config_.red_min_v);
  max_hsv_red_ = cv::Scalar(hsv_config_.red_max_h, hsv_config_.red_max_s, hsv_config_.red_max_v);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace traffic_light
