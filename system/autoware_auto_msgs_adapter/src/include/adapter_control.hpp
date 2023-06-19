// Copyright 2023 The Autoware Foundation
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
#ifndef ADAPTER_CONTROL_HPP_
#define ADAPTER_CONTROL_HPP_

#include "adapter_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_control_msgs/msg/control.hpp>

#include <string>

namespace autoware_auto_msgs_adapter
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_control_msgs::msg::Control;

class AdapterControl
: public autoware_auto_msgs_adapter::AdapterBase<Control, AckermannControlCommand>
{
public:
  AdapterControl(
    rclcpp::Node & node, const std::string & topic_name_source,
    const std::string & topic_name_target, const rclcpp::QoS & qos = rclcpp::QoS{1})
  : AdapterBase(node, topic_name_source, topic_name_target, qos)
  {
    RCLCPP_DEBUG(
      node.get_logger(), "AdapterControl is initialized to convert: %s -> %s",
      topic_name_source.c_str(), topic_name_target.c_str());
  }

protected:
  AckermannControlCommand convert(const Control & msg_source) override
  {
    autoware_auto_control_msgs::msg::AckermannControlCommand msg_auto;
    msg_auto.stamp = msg_source.stamp;

    const auto & lateral = msg_source.lateral;
    auto & lateral_auto = msg_auto.lateral;
    lateral_auto.stamp = lateral.stamp;
    lateral_auto.steering_tire_angle = lateral.steering_tire_angle;
    lateral_auto.steering_tire_rotation_rate = lateral.steering_tire_rotation_rate;

    const auto & longitudinal = msg_source.longitudinal;
    auto & longitudinal_auto = msg_auto.longitudinal;
    longitudinal_auto.stamp = longitudinal.stamp;
    longitudinal_auto.acceleration = longitudinal.acceleration;
    longitudinal_auto.jerk = longitudinal.jerk;
    longitudinal_auto.speed = longitudinal.velocity;

    return msg_auto;
  }
};
}  // namespace autoware_auto_msgs_adapter

#endif  // ADAPTER_CONTROL_HPP_
