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

#include "autoware/shift_decider/autoware_shift_decider.hpp"

#include <rclcpp/timer.hpp>

#include <cstddef>
#include <functional>

namespace autoware::shift_decider
{

ShiftDecider::ShiftDecider(const rclcpp::NodeOptions & node_options)
: Node("shift_decider", node_options)
{
  using std::placeholders::_1;

  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS durable_qos(queue_size);
  durable_qos.transient_local();

  park_on_goal_ = declare_parameter<bool>("park_on_goal");

  pub_shift_cmd_ =
    create_publisher<autoware_vehicle_msgs::msg::GearCommand>("output/gear_cmd", durable_qos);

  initTimer(0.1);
}

void ShiftDecider::onTimer()
{
  control_cmd_ = sub_control_cmd_.takeData();
  autoware_state_ = sub_autoware_state_.takeData();
  current_gear_ptr_ = sub_current_gear_.takeData();
  if (!autoware_state_ || !control_cmd_ || !current_gear_ptr_) {
    return;
  }

  updateCurrentShiftCmd();
  pub_shift_cmd_->publish(shift_cmd_);
}

void ShiftDecider::updateCurrentShiftCmd()
{
  using autoware_system_msgs::msg::AutowareState;
  using autoware_vehicle_msgs::msg::GearCommand;

  shift_cmd_.stamp = now();
  static constexpr double vel_threshold = 0.01;  // to prevent chattering
  if (autoware_state_->state == AutowareState::DRIVING) {
    if (control_cmd_->longitudinal.velocity > vel_threshold) {
      shift_cmd_.command = GearCommand::DRIVE;
    } else if (control_cmd_->longitudinal.velocity < -vel_threshold) {
      shift_cmd_.command = GearCommand::REVERSE;
    } else {
      shift_cmd_.command = prev_shift_command;
    }
  } else {
    if (
      (autoware_state_->state == AutowareState::ARRIVED_GOAL ||
       autoware_state_->state == AutowareState::WAITING_FOR_ROUTE) &&
      park_on_goal_) {
      shift_cmd_.command = GearCommand::PARK;
    } else {
      shift_cmd_.command = current_gear_ptr_->report;
    }
  }
  prev_shift_command = shift_cmd_.command;
}

void ShiftDecider::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&ShiftDecider::onTimer, this));
}
}  // namespace autoware::shift_decider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::shift_decider::ShiftDecider)
