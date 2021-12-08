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

#include "awapi_awiv_adapter/awapi_pacmod_util.hpp"

namespace autoware_api
{
namespace pacmod_util
{
autoware_api_msgs::msg::DoorStatus getDoorStatusMsg(
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr & msg_ptr)
{
  using autoware_api_msgs::msg::DoorStatus;
  using pacmod3_msgs::msg::SystemRptInt;
  DoorStatus door_status;

  if (!msg_ptr) {
    door_status.status = DoorStatus::NOT_APPLICABLE;
    return door_status;
  }

  door_status.status = DoorStatus::UNKNOWN;

  if (msg_ptr->command == SystemRptInt::DOOR_CLOSE && msg_ptr->output == SystemRptInt::DOOR_OPEN) {
    // do not used (command & output are always the same value)
    door_status.status = DoorStatus::DOOR_CLOSING;
  } else if (  // NOLINT
    msg_ptr->command == SystemRptInt::DOOR_OPEN && msg_ptr->output == SystemRptInt::DOOR_CLOSE) {
    // do not used (command & output are always the same value)
    door_status.status = DoorStatus::DOOR_OPENING;
  } else if (msg_ptr->output == SystemRptInt::DOOR_CLOSE) {
    door_status.status = DoorStatus::DOOR_CLOSED;
  } else if (msg_ptr->output == SystemRptInt::DOOR_OPEN) {
    door_status.status = DoorStatus::DOOR_OPENED;
  }

  return door_status;
}

pacmod3_msgs::msg::SystemCmdInt createClearOverrideDoorCommand(
  const rclcpp::Clock::SharedPtr & clock)
{
  pacmod3_msgs::msg::SystemCmdInt door_cmd;
  door_cmd.header.frame_id = "base_link";
  door_cmd.header.stamp = clock->now();
  door_cmd.clear_override = true;
  return door_cmd;
}

pacmod3_msgs::msg::SystemCmdInt createDoorCommand(
  const rclcpp::Clock::SharedPtr & clock,
  const autoware_api_msgs::msg::DoorControlCommand::ConstSharedPtr & msg_ptr)
{
  using pacmod3_msgs::msg::SystemCmdInt;

  SystemCmdInt door_cmd;
  door_cmd.header.frame_id = "base_link";
  door_cmd.header.stamp = clock->now();
  door_cmd.enable = true;
  door_cmd.command = SystemCmdInt::DOOR_NEUTRAL;

  if (!msg_ptr) {
    return door_cmd;
  }

  if (msg_ptr->open) {
    door_cmd.command = SystemCmdInt::DOOR_OPEN;
  } else {
    door_cmd.command = SystemCmdInt::DOOR_CLOSE;
  }
  return door_cmd;
}

}  // namespace pacmod_util

}  // namespace autoware_api
