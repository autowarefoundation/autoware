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

#ifndef SIMPLE_OBJECT_MERGER__SIMPLE_OBJECT_MERGER_NODE_HPP_
#define SIMPLE_OBJECT_MERGER__SIMPLE_OBJECT_MERGER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace simple_object_merger
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

class SimpleObjectMergerNode : public rclcpp::Node
{
public:
  explicit SimpleObjectMergerNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
    double timeout_threshold{};
    std::vector<std::string> topic_names{};
    std::string new_frame_id{};
  };

private:
  // Subscriber
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_{};
  std::vector<rclcpp::Subscription<DetectedObjects>::SharedPtr> sub_objects_array{};
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  // Callback
  void onData(const DetectedObjects::ConstSharedPtr msg, size_t array_number);

  // Data Buffer
  std::vector<DetectedObjects::ConstSharedPtr> objects_data_{};
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_{};

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};
  void onTimer();
  bool isDataReady();

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Core
  size_t input_topic_size;
};

}  // namespace simple_object_merger

#endif  // SIMPLE_OBJECT_MERGER__SIMPLE_OBJECT_MERGER_NODE_HPP_
