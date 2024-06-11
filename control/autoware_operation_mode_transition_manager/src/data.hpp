// Copyright 2022 Autoware Foundation
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

#ifndef DATA_HPP_
#define DATA_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_system_msgs/srv/change_operation_mode.hpp>

#include <cmath>
#include <optional>
#include <string>

namespace autoware::operation_mode_transition_manager
{

using ServiceResponse = autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response;
using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
using OperationModeValue = OperationModeState::_mode_type;
using ChangeOperationMode = tier4_system_msgs::srv::ChangeOperationMode;
using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;
using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;

enum class OperationMode { STOP, AUTONOMOUS, LOCAL, REMOTE };

struct Transition
{
  Transition(const rclcpp::Time & time, bool control, std::optional<OperationMode> previous)
  : time(time), previous(previous)
  {
    is_engage_requested = control;
    is_engage_failed = false;
    is_engage_completed = false;
  }

  const rclcpp::Time time;
  const std::optional<OperationMode> previous;
  bool is_engage_requested;
  bool is_engage_failed;
  bool is_engage_completed;
};

struct EngageAcceptableParam
{
  bool allow_autonomous_in_stopped = true;
  double dist_threshold = 2.0;
  double speed_upper_threshold = 10.0;
  double speed_lower_threshold = -10.0;
  double yaw_threshold = 0.785;
  double acc_threshold = 2.0;
  double lateral_acc_threshold = 2.0;
  double lateral_acc_diff_threshold = 1.0;
};

struct StableCheckParam
{
  double duration = 3.0;
  double dist_threshold = 0.5;
  double speed_upper_threshold = 3.0;
  double speed_lower_threshold = -3.0;
  double yaw_threshold = M_PI / 10.0;
};

std::string toString(const std::optional<OperationMode> mode);
std::optional<OperationMode> toEnum(const ChangeOperationMode::Request & request);
OperationModeValue toMsg(const OperationMode mode);

}  // namespace autoware::operation_mode_transition_manager

#endif  // DATA_HPP_
