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

#ifndef RTC_AUTO_MODE_MANAGER__RTC_AUTO_MODE_MANAGER_INTERFACE_HPP_
#define RTC_AUTO_MODE_MANAGER__RTC_AUTO_MODE_MANAGER_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_rtc_msgs/srv/auto_mode.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rtc_auto_mode_manager
{
using tier4_rtc_msgs::srv::AutoMode;

class RTCAutoModeManagerInterface
{
public:
  RTCAutoModeManagerInterface(
    rclcpp::Node * node, const std::string & module_name, const bool default_enable);

private:
  void onEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response) const;
  AutoMode::Request createRequest(const AutoMode::Request::SharedPtr request) const;

  rclcpp::Client<AutoMode>::SharedPtr enable_cli_;
  rclcpp::Service<AutoMode>::SharedPtr enable_srv_;

  std::string enable_auto_mode_namespace_ = "/planning/enable_auto_mode";
};
}  // namespace rtc_auto_mode_manager

#endif  // RTC_AUTO_MODE_MANAGER__RTC_AUTO_MODE_MANAGER_INTERFACE_HPP_
