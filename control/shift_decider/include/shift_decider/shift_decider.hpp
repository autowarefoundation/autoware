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

#ifndef SHIFT_DECIDER__SHIFT_DECIDER_HPP_
#define SHIFT_DECIDER__SHIFT_DECIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>

#include <memory>

class ShiftDecider : public rclcpp::Node
{
public:
  explicit ShiftDecider(const rclcpp::NodeOptions & node_options);

private:
  void onTimer();
  void onControlCmd(autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
  void onAutowareState(autoware_auto_system_msgs::msg::AutowareState::SharedPtr msg);
  void onCurrentGear(autoware_auto_vehicle_msgs::msg::GearReport::SharedPtr msg);
  void updateCurrentShiftCmd();
  void initTimer(double period_s);

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr pub_shift_cmd_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_control_cmd_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr
    sub_autoware_state_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr sub_current_gear_;

  rclcpp::TimerBase::SharedPtr timer_;

  autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr control_cmd_;
  autoware_auto_system_msgs::msg::AutowareState::SharedPtr autoware_state_;
  autoware_auto_vehicle_msgs::msg::GearCommand shift_cmd_;
  autoware_auto_vehicle_msgs::msg::GearReport::SharedPtr current_gear_ptr_;
  uint8_t prev_shift_command = autoware_auto_vehicle_msgs::msg::GearCommand::PARK;

  bool park_on_goal_;
};

#endif  // SHIFT_DECIDER__SHIFT_DECIDER_HPP_
