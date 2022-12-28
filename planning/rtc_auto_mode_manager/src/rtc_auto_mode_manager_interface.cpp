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
Module getModuleType(const std::string & module_name)
{
  Module module;
  if (module_name == "blind_spot") {
    module.type = Module::BLIND_SPOT;
  } else if (module_name == "crosswalk") {
    module.type = Module::CROSSWALK;
  } else if (module_name == "detection_area") {
    module.type = Module::DETECTION_AREA;
  } else if (module_name == "intersection") {
    module.type = Module::INTERSECTION;
  } else if (module_name == "no_stopping_area") {
    module.type = Module::NO_STOPPING_AREA;
  } else if (module_name == "occlusion_spot") {
    module.type = Module::OCCLUSION_SPOT;
  } else if (module_name == "traffic_light") {
    module.type = Module::TRAFFIC_LIGHT;
  } else if (module_name == "virtual_traffic_light") {
    module.type = Module::TRAFFIC_LIGHT;
  } else if (module_name == "lane_change_left") {
    module.type = Module::LANE_CHANGE_LEFT;
  } else if (module_name == "lane_change_right") {
    module.type = Module::LANE_CHANGE_RIGHT;
  } else if (module_name == "avoidance_left") {
    module.type = Module::AVOIDANCE_LEFT;
  } else if (module_name == "avoidance_right") {
    module.type = Module::AVOIDANCE_RIGHT;
  } else if (module_name == "pull_over") {
    module.type = Module::PULL_OVER;
  } else if (module_name == "pull_out") {
    module.type = Module::PULL_OUT;
  } else {
    module.type = Module::NONE;
  }
  return module;
}

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

  auto_mode_status_.module = getModuleType(module_name);
  // Send enable auto mode request
  if (default_enable) {
    auto_mode_status_.is_auto_mode = true;
    AutoMode::Request::SharedPtr request = std::make_shared<AutoMode::Request>();
    request->enable = true;
    enable_cli_->async_send_request(request);
  }
}

void RTCAutoModeManagerInterface::onEnableService(
  const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response)
{
  auto_mode_status_.is_auto_mode = request->enable;
  enable_cli_->async_send_request(request);
  response->success = true;
}

}  // namespace rtc_auto_mode_manager
