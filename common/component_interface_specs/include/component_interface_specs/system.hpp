// Copyright 2022 TIER IV, Inc.
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

#ifndef COMPONENT_INTERFACE_SPECS__SYSTEM_HPP_
#define COMPONENT_INTERFACE_SPECS__SYSTEM_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_system_msgs/srv/change_autoware_control.hpp>
#include <tier4_system_msgs/srv/change_operation_mode.hpp>

namespace system_interface
{

struct MrmState
{
  using Message = autoware_adapi_v1_msgs::msg::MrmState;
  static constexpr char name[] = "/system/fail_safe/mrm_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct ChangeAutowareControl
{
  using Service = tier4_system_msgs::srv::ChangeAutowareControl;
  static constexpr char name[] = "/system/operation_mode/change_autoware_control";
};

struct ChangeOperationMode
{
  using Service = tier4_system_msgs::srv::ChangeOperationMode;
  static constexpr char name[] = "/system/operation_mode/change_operation_mode";
};

struct OperationModeState
{
  using Message = autoware_adapi_v1_msgs::msg::OperationModeState;
  static constexpr char name[] = "/system/operation_mode/state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace system_interface

#endif  // COMPONENT_INTERFACE_SPECS__SYSTEM_HPP_
