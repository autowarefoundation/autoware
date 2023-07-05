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

#ifndef OBJECT_VELOCITY_SPLITTER__OBJECT_VELOCITY_SPLITTER_NODE_HPP_
#define OBJECT_VELOCITY_SPLITTER__OBJECT_VELOCITY_SPLITTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace object_velocity_splitter
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;

class ObjectVelocitySplitterNode : public rclcpp::Node
{
public:
  explicit ObjectVelocitySplitterNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double velocity_threshold{};
  };

private:
  // Subscriber
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_{};

  // Callback
  void onObjects(const DetectedObjects::ConstSharedPtr msg);

  // Data Buffer
  DetectedObjects::ConstSharedPtr objects_data_{};

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_high_speed_objects_{};
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_low_speed_objects_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};
};

}  // namespace object_velocity_splitter

#endif  // OBJECT_VELOCITY_SPLITTER__OBJECT_VELOCITY_SPLITTER_NODE_HPP_
