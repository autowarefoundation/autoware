// Copyright 2022 Tier IV, Inc.
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

#ifndef RTC_INTERFACE__RTC_INTERFACE_HPP_
#define RTC_INTERFACE__RTC_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_rtc_msgs/msg/auto_mode_status.hpp"
#include "tier4_rtc_msgs/msg/command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_response.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include "tier4_rtc_msgs/srv/auto_mode.hpp"
#include "tier4_rtc_msgs/srv/cooperate_commands.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace rtc_interface
{
using tier4_rtc_msgs::msg::AutoModeStatus;
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateCommand;
using tier4_rtc_msgs::msg::CooperateResponse;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::srv::AutoMode;
using tier4_rtc_msgs::srv::CooperateCommands;
using unique_identifier_msgs::msg::UUID;

class RTCInterface
{
public:
  RTCInterface(rclcpp::Node * node, const std::string & name, const bool enable_rtc = true);
  void publishCooperateStatus(const rclcpp::Time & stamp);
  void updateCooperateStatus(
    const UUID & uuid, const bool safe, const double start_distance, const double finish_distance,
    const rclcpp::Time & stamp);
  void removeCooperateStatus(const UUID & uuid);
  void clearCooperateStatus();
  bool isActivated(const UUID & uuid) const;
  bool isRegistered(const UUID & uuid) const;
  bool isRTCEnabled(const UUID & uuid) const;
  void lockCommandUpdate();
  void unlockCommandUpdate();

private:
  void onCooperateCommandService(
    const CooperateCommands::Request::SharedPtr request,
    const CooperateCommands::Response::SharedPtr responses);
  void onAutoModeService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onTimer();
  std::vector<CooperateResponse> validateCooperateCommands(
    const std::vector<CooperateCommand> & commands);
  void updateCooperateCommandStatus(const std::vector<CooperateCommand> & commands);
  void removeStoredCommand(const UUID & uuid);
  rclcpp::Logger getLogger() const;
  bool isLocked() const;

  rclcpp::Publisher<CooperateStatusArray>::SharedPtr pub_statuses_;
  rclcpp::Publisher<AutoModeStatus>::SharedPtr pub_auto_mode_status_;
  rclcpp::Service<CooperateCommands>::SharedPtr srv_commands_;
  rclcpp::Service<AutoMode>::SharedPtr srv_auto_mode_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Logger logger_;

  Module module_;
  CooperateStatusArray registered_status_;
  std::vector<CooperateCommand> stored_commands_;
  bool is_auto_mode_enabled_;
  bool is_locked_;

  std::string cooperate_status_namespace_ = "/planning/cooperate_status";
  std::string auto_mode_status_namespace_ = "/planning/auto_mode_status";
  std::string cooperate_commands_namespace_ = "/planning/cooperate_commands";
  std::string enable_auto_mode_namespace_ = "/planning/enable_auto_mode";

  mutable std::mutex mutex_;
};

}  // namespace rtc_interface

#endif  // RTC_INTERFACE__RTC_INTERFACE_HPP_
