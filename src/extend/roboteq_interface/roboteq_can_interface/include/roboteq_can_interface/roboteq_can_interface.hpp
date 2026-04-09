// Copyright 2021 Tier IV, Inc.
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

#ifndef ROBOTEQ_CAN_INTERFACE__ROBOTEQ_CAN_INTERFACE_HPP_
#define ROBOTEQ_CAN_INTERFACE__ROBOTEQ_CAN_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "roboteq_can_interface/can_command_data.hpp"

#include "can_msgs/msg/frame.hpp"
#include "roboteq_msgs/msg/roboteq_command_stamped.hpp"
#include "roboteq_msgs/msg/roboteq_status_stamped.hpp"

class RoboteqCanInterface : public rclcpp::Node
{
public:
  explicit RoboteqCanInterface(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  int node_id_;
  // variables
  CanCommandData can_cmd_;
  roboteq_msgs::msg::RoboteqStatusStamped status_;

  // Callback
  void onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void onRoboteqCommand(const roboteq_msgs::msg::RoboteqCommandStamped::ConstSharedPtr msg);

  void publishEmergencyRelease();
  void publishEmergencyStop();

  // Subscriber
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  rclcpp::Subscription<roboteq_msgs::msg::RoboteqCommandStamped>::SharedPtr command_sub_;

  // Publisher
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Publisher<roboteq_msgs::msg::RoboteqStatusStamped>::SharedPtr status_pub_;
};

#endif  // ROBOTEQ_CAN_INTERFACE__ROBOTEQ_CAN_INTERFACE_HPP_
