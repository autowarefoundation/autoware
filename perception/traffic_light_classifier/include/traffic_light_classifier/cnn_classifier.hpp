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

#ifndef TRAFFIC_LIGHT_CLASSIFIER__CNN_CLASSIFIER_HPP_
#define TRAFFIC_LIGHT_CLASSIFIER__CNN_CLASSIFIER_HPP_

#include "traffic_light_classifier/classifier_interface.hpp"

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trt_common.hpp>

#include <autoware_auto_perception_msgs/msg/traffic_light.hpp>

#include <cv_bridge/cv_bridge.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace traffic_light
{
class CNNClassifier : public ClassifierInterface
{
public:
  explicit CNNClassifier(rclcpp::Node * node_ptr);
  virtual ~CNNClassifier() = default;

  bool getTrafficSignal(
    const cv::Mat & input_image,
    autoware_auto_perception_msgs::msg::TrafficSignal & traffic_signal) override;

private:
  void preProcess(cv::Mat & image, std::vector<float> & tensor, bool normalize = true);
  bool postProcess(
    std::vector<float> & output_data_host,
    autoware_auto_perception_msgs::msg::TrafficSignal & traffic_signal);
  bool readLabelfile(std::string filepath, std::vector<std::string> & labels);
  bool isColorLabel(const std::string label);
  void calcSoftmax(std::vector<float> & data, std::vector<float> & probs, int num_output);
  std::vector<size_t> argsort(std::vector<float> & tensor, int num_output);
  void outputDebugImage(
    cv::Mat & debug_image,
    const autoware_auto_perception_msgs::msg::TrafficSignal & traffic_signal);

private:
  std::map<int, std::string> state2label_{
    // color
    {autoware_auto_perception_msgs::msg::TrafficLight::RED, "red"},
    {autoware_auto_perception_msgs::msg::TrafficLight::AMBER, "yellow"},
    {autoware_auto_perception_msgs::msg::TrafficLight::GREEN, "green"},
    {autoware_auto_perception_msgs::msg::TrafficLight::WHITE, "white"},
    // shape
    {autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE, "circle"},
    {autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW, "left"},
    {autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW, "right"},
    {autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW, "straight"},
    {autoware_auto_perception_msgs::msg::TrafficLight::DOWN_ARROW, "down"},
    {autoware_auto_perception_msgs::msg::TrafficLight::DOWN_LEFT_ARROW, "down_left"},
    {autoware_auto_perception_msgs::msg::TrafficLight::DOWN_RIGHT_ARROW, "down_right"},
    {autoware_auto_perception_msgs::msg::TrafficLight::CROSS, "cross"},
    // other
    {autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN, "unknown"},
  };

  std::map<std::string, int> label2state_{
    // color
    {"red", autoware_auto_perception_msgs::msg::TrafficLight::RED},
    {"yellow", autoware_auto_perception_msgs::msg::TrafficLight::AMBER},
    {"green", autoware_auto_perception_msgs::msg::TrafficLight::GREEN},
    {"white", autoware_auto_perception_msgs::msg::TrafficLight::WHITE},
    // shape
    {"circle", autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE},
    {"left", autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW},
    {"right", autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW},
    {"straight", autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW},
    {"down", autoware_auto_perception_msgs::msg::TrafficLight::DOWN_ARROW},
    {"down_left", autoware_auto_perception_msgs::msg::TrafficLight::DOWN_LEFT_ARROW},
    {"down_right", autoware_auto_perception_msgs::msg::TrafficLight::DOWN_RIGHT_ARROW},
    {"cross", autoware_auto_perception_msgs::msg::TrafficLight::CROSS},
    // other
    {"unknown", autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN},
  };

  rclcpp::Node * node_ptr_;

  std::shared_ptr<Tn::TrtCommon> trt_;
  image_transport::Publisher image_pub_;
  std::vector<std::string> labels_;
  std::vector<float> mean_{0.242, 0.193, 0.201};
  std::vector<float> std_{1.0, 1.0, 1.0};
  int input_c_;
  int input_h_;
  int input_w_;
};

}  // namespace traffic_light

#endif  // TRAFFIC_LIGHT_CLASSIFIER__CNN_CLASSIFIER_HPP_
