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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_PACMOD_UTIL_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_PACMOD_UTIL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <pacmod3_msgs/msg/system_cmd_int.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <tier4_api_msgs/msg/door_control_command.hpp>
#include <tier4_api_msgs/msg/door_status.hpp>

namespace autoware_api
{
namespace pacmod_util
{
tier4_api_msgs::msg::DoorStatus getDoorStatusMsg(
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr & msg_ptr);
pacmod3_msgs::msg::SystemCmdInt createClearOverrideDoorCommand(
  const rclcpp::Clock::SharedPtr & clock);
pacmod3_msgs::msg::SystemCmdInt createDoorCommand(
  const rclcpp::Clock::SharedPtr & clock,
  const tier4_api_msgs::msg::DoorControlCommand::ConstSharedPtr & msg_ptr);
}  // namespace pacmod_util

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_PACMOD_UTIL_HPP_
