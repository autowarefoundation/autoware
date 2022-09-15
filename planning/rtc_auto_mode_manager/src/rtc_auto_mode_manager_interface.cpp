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

#include "rtc_auto_mode_manager/rtc_auto_mode_manager_interface.hpp"

namespace rtc_auto_mode_manager
{
RTCAutoModeManagerInterface::RTCAutoModeManagerInterface(
  rclcpp::Node * node, const std::string & module_name, const bool default_enable)
{
  using std::chrono_literals::operator""s;
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Service client
  enable_cli_ = node->create_client<AutoMode>(
    enable_auto_mode_namespace_ + "/internal/" + module_name, rmw_qos_profile_services_default);

  while (!enable_cli_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Waiting for service... [" << module_name << "]");
  }

  // Service
  enable_srv_ = node->create_service<AutoMode>(
    enable_auto_mode_namespace_ + "/" + module_name,
    std::bind(&RTCAutoModeManagerInterface::onEnableService, this, _1, _2));

  // Send enable auto mode request
  if (default_enable) {
    AutoMode::Request::SharedPtr request = std::make_shared<AutoMode::Request>();
    request->enable = true;
    enable_cli_->async_send_request(request);
  }
}

void RTCAutoModeManagerInterface::onEnableService(
  const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response) const
{
  enable_cli_->async_send_request(request);
  response->success = true;
}

}  // namespace rtc_auto_mode_manager
