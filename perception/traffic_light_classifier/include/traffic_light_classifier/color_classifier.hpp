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

#ifndef TRAFFIC_LIGHT_CLASSIFIER__COLOR_CLASSIFIER_HPP_
#define TRAFFIC_LIGHT_CLASSIFIER__COLOR_CLASSIFIER_HPP_

#include "traffic_light_classifier/classifier_interface.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/traffic_signal_array.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <vector>

namespace traffic_light
{
struct HSVConfig
{
  int green_min_h;
  int green_min_s;
  int green_min_v;
  int green_max_h;
  int green_max_s;
  int green_max_v;
  int yellow_min_h;
  int yellow_min_s;
  int yellow_min_v;
  int yellow_max_h;
  int yellow_max_s;
  int yellow_max_v;
  int red_min_h;
  int red_min_s;
  int red_min_v;
  int red_max_h;
  int red_max_s;
  int red_max_v;
};

class ColorClassifier : public ClassifierInterface
{
public:
  explicit ColorClassifier(rclcpp::Node * node_ptr);
  virtual ~ColorClassifier() = default;

  bool getTrafficSignals(
    const std::vector<cv::Mat> & images,
    tier4_perception_msgs::msg::TrafficSignalArray & traffic_signals) override;

private:
  bool filterHSV(
    const cv::Mat & input_image, cv::Mat & green_image, cv::Mat & yellow_image,
    cv::Mat & red_image);
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

private:
  enum HSV {
    Hue = 0,
    Sat = 1,
    Val = 2,
  };
  image_transport::Publisher image_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rclcpp::Node * node_ptr_;

  HSVConfig hsv_config_;
  cv::Scalar min_hsv_green_;
  cv::Scalar max_hsv_green_;
  cv::Scalar min_hsv_yellow_;
  cv::Scalar max_hsv_yellow_;
  cv::Scalar min_hsv_red_;
  cv::Scalar max_hsv_red_;
};

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_CLASSIFIER__COLOR_CLASSIFIER_HPP_
