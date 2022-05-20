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

#include "rtc_interface/rtc_interface.hpp"

namespace
{
using tier4_rtc_msgs::msg::Module;

std::string to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}

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
  } else if (module_name == "stop_line") {
    module.type = Module::NONE;
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
  }
  return module;
}

}  // namespace

namespace rtc_interface
{
RTCInterface::RTCInterface(rclcpp::Node & node, const std::string & name)
: logger_{node.get_logger().get_child("RTCInterface[" + name + "]")}
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Publisher
  pub_statuses_ = node.create_publisher<CooperateStatusArray>("~/" + name + "/cooperate_status", 1);

  // Service
  srv_commands_ = node.create_service<CooperateCommands>(
    "~/" + name + "/cooperate_commands",
    std::bind(&RTCInterface::onCooperateCommandService, this, _1, _2));

  // Module
  module_ = getModuleType(name);
}

void RTCInterface::publishCooperateStatus(const rclcpp::Time & stamp)
{
  registered_status_.stamp = stamp;
  pub_statuses_->publish(registered_status_);
}

void RTCInterface::onCooperateCommandService(
  const CooperateCommands::Request::SharedPtr request,
  const CooperateCommands::Response::SharedPtr responses)
{
  for (const auto & command : request->commands) {
    CooperateResponse response;
    response.uuid = command.uuid;
    response.module = command.module;

    const auto itr = std::find_if(
      registered_status_.statuses.begin(), registered_status_.statuses.end(),
      [command](auto & s) { return s.uuid == command.uuid; });

    // Update command if the command has been already received
    if (itr != registered_status_.statuses.end()) {
      itr->command_status = command.command;
      response.success = true;
    } else {
      RCLCPP_WARN_STREAM(
        getLogger(), "[onCooperateCommandService] uuid : " << to_string(command.uuid)
                                                           << " is not found." << std::endl);
      response.success = false;
    }
    responses->responses.push_back(response);
  }
}

void RTCInterface::updateCooperateStatus(
  const UUID & uuid, const bool safe, const double distance, const rclcpp::Time & stamp)
{
  // Find registered status which has same uuid
  auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  // If there is no registered status, add it
  if (itr == registered_status_.statuses.end()) {
    CooperateStatus status;
    status.stamp = stamp;
    status.uuid = uuid;
    status.module = module_;
    status.safe = safe;
    status.command_status.type = Command::DEACTIVATE;
    status.distance = distance;
    registered_status_.statuses.push_back(status);
    return;
  }

  // If the registered status is found, update status
  itr->stamp = stamp;
  itr->safe = safe;
  itr->distance = distance;
}

void RTCInterface::removeCooperateStatus(const UUID & uuid)
{
  // Find registered status which has same uuid and erase it
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    registered_status_.statuses.erase(itr);
    return;
  }

  RCLCPP_WARN_STREAM(
    getLogger(),
    "[removeCooperateStatus] uuid : " << to_string(uuid) << " is not found." << std::endl);
}

void RTCInterface::clearCooperateStatus() { registered_status_.statuses.clear(); }

bool RTCInterface::isActivated(const UUID & uuid) const
{
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    return itr->command_status.type == Command::ACTIVATE;
  }

  RCLCPP_WARN_STREAM(
    getLogger(), "[isActivated] uuid : " << to_string(uuid) << " is not found." << std::endl);
  return false;
}

bool RTCInterface::isRegistered(const UUID & uuid) const
{
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });
  return itr != registered_status_.statuses.end();
}

rclcpp::Logger RTCInterface::getLogger() const { return logger_; }

}  // namespace rtc_interface
