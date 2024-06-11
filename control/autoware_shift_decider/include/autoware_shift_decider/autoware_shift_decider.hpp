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

#ifndef AUTOWARE_SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_
#define AUTOWARE_SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_

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
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr sub_autoware_state_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr sub_current_gear_;

  rclcpp::TimerBase::SharedPtr timer_;

  autoware_control_msgs::msg::Control::SharedPtr control_cmd_;
  autoware_system_msgs::msg::AutowareState::SharedPtr autoware_state_;
  autoware_vehicle_msgs::msg::GearCommand shift_cmd_;
  autoware_vehicle_msgs::msg::GearReport::SharedPtr current_gear_ptr_;
  uint8_t prev_shift_command = autoware_vehicle_msgs::msg::GearCommand::PARK;

  bool park_on_goal_;
};
}  // namespace autoware::shift_decider

#endif  // AUTOWARE_SHIFT_DECIDER__AUTOWARE_SHIFT_DECIDER_HPP_
