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

#ifndef RTC_AUTO_APPROVER__RTC_AUTO_APPROVER_INTERFACE_HPP_
#define RTC_AUTO_APPROVER__RTC_AUTO_APPROVER_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_rtc_msgs/msg/command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_response.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include "tier4_rtc_msgs/srv/auto_mode.hpp"
#include "tier4_rtc_msgs/srv/cooperate_commands.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>
#include <vector>

namespace rtc_auto_approver
{
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateCommand;
using tier4_rtc_msgs::msg::CooperateResponse;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::srv::AutoMode;
using tier4_rtc_msgs::srv::CooperateCommands;
using unique_identifier_msgs::msg::UUID;

class RTCAutoApproverInterface
{
public:
  RTCAutoApproverInterface(
    rclcpp::Node * node, const std::string & name, const bool default_enable);

private:
  void onEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onStatus(const CooperateStatusArray::ConstSharedPtr msg) const;
  bool isNecessarySendCommand(const CooperateStatus & status) const;
  CooperateCommands::Request createRequest(const CooperateStatusArray & array) const;

  rclcpp::Subscription<CooperateStatusArray>::SharedPtr status_sub_;
  rclcpp::Client<CooperateCommands>::SharedPtr command_cli_;
  rclcpp::Service<AutoMode>::SharedPtr enable_srv_;

  bool enabled_;
};
}  // namespace rtc_auto_approver

#endif  // RTC_AUTO_APPROVER__RTC_AUTO_APPROVER_INTERFACE_HPP_
