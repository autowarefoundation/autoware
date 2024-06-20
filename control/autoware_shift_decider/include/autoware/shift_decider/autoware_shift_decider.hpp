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

#ifndef AUTOWARE__SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_
#define AUTOWARE__SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_

#include "autoware/universe_utils/ros/polling_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>

#include <memory>

namespace autoware::shift_decider
{

class ShiftDecider : public rclcpp::Node
{
public:
  explicit ShiftDecider(const rclcpp::NodeOptions & node_options);

private:
  void onTimer();
  void onControlCmd(autoware_control_msgs::msg::Control::SharedPtr msg);
  void onAutowareState(autoware_system_msgs::msg::AutowareState::SharedPtr msg);
  void onCurrentGear(autoware_vehicle_msgs::msg::GearReport::SharedPtr msg);
  void updateCurrentShiftCmd();
  void initTimer(double period_s);

  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_shift_cmd_;
  autoware::universe_utils::InterProcessPollingSubscriber<autoware_control_msgs::msg::Control>
    sub_control_cmd_{this, "input/control_cmd"};
  autoware::universe_utils::InterProcessPollingSubscriber<autoware_system_msgs::msg::AutowareState>
    sub_autoware_state_{this, "input/state"};
  autoware::universe_utils::InterProcessPollingSubscriber<autoware_vehicle_msgs::msg::GearReport>
    sub_current_gear_{this, "input/current_gear"};

  rclcpp::TimerBase::SharedPtr timer_;

  autoware_control_msgs::msg::Control::ConstSharedPtr control_cmd_;
  autoware_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state_;
  autoware_vehicle_msgs::msg::GearCommand shift_cmd_;
  autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr current_gear_ptr_;
  uint8_t prev_shift_command = autoware_vehicle_msgs::msg::GearCommand::PARK;

  bool park_on_goal_;
};
}  // namespace autoware::shift_decider

#endif  // AUTOWARE__SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_
