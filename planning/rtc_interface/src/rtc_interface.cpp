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
RTCInterface::RTCInterface(rclcpp::Node * node, const std::string & name)
: logger_{node->get_logger().get_child("RTCInterface[" + name + "]")},
  is_auto_mode_{false},
  is_locked_{false}
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Publisher
  pub_statuses_ =
    node->create_publisher<CooperateStatusArray>(cooperate_status_namespace_ + "/" + name, 1);

  // Service
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_commands_ = node->create_service<CooperateCommands>(
    cooperate_commands_namespace_ + "/" + name,
    std::bind(&RTCInterface::onCooperateCommandService, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_);
  srv_auto_mode_ = node->create_service<AutoMode>(
    enable_auto_mode_namespace_ + "/" + name,
    std::bind(&RTCInterface::onAutoModeService, this, _1, _2), rmw_qos_profile_services_default,
    callback_group_);

  // Module
  module_ = getModuleType(name);
}

void RTCInterface::publishCooperateStatus(const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  registered_status_.stamp = stamp;
  pub_statuses_->publish(registered_status_);
}

void RTCInterface::onCooperateCommandService(
  const CooperateCommands::Request::SharedPtr request,
  const CooperateCommands::Response::SharedPtr responses)
{
  std::lock_guard<std::mutex> lock(mutex_);

  responses->responses = validateCooperateCommands(request->commands);

  if (isLocked()) {
    stored_commands_ = request->commands;
    return;
  }

  updateCooperateCommandStatus(request->commands);
}

std::vector<CooperateResponse> RTCInterface::validateCooperateCommands(
  const std::vector<CooperateCommand> & commands)
{
  std::vector<CooperateResponse> responses;

  for (const auto & command : commands) {
    CooperateResponse response;
    response.uuid = command.uuid;
    response.module = command.module;

    const auto itr = std::find_if(
      registered_status_.statuses.begin(), registered_status_.statuses.end(),
      [command](auto & s) { return s.uuid == command.uuid; });
    if (itr != registered_status_.statuses.end()) {
      response.success = true;
    } else {
      RCLCPP_WARN_STREAM(
        getLogger(), "[validateCooperateCommands] uuid : " << to_string(command.uuid)
                                                           << " is not found." << std::endl);
      response.success = false;
    }
    responses.push_back(response);
  }

  return responses;
}

void RTCInterface::updateCooperateCommandStatus(const std::vector<CooperateCommand> & commands)
{
  for (const auto & command : commands) {
    const auto itr = std::find_if(
      registered_status_.statuses.begin(), registered_status_.statuses.end(),
      [command](auto & s) { return s.uuid == command.uuid; });

    // Update command if the command has been already received
    if (itr != registered_status_.statuses.end()) {
      itr->command_status = command.command;
      itr->auto_mode = false;
      is_auto_mode_ = false;
    }
  }
}

void RTCInterface::onAutoModeService(
  const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_auto_mode_ = request->enable;
  for (auto & status : registered_status_.statuses) {
    status.auto_mode = request->enable;
  }
  response->success = true;
}

void RTCInterface::updateCooperateStatus(
  const UUID & uuid, const bool safe, const double start_distance, const double finish_distance,
  const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
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
    status.start_distance = start_distance;
    status.finish_distance = finish_distance;
    status.auto_mode = is_auto_mode_;
    registered_status_.statuses.push_back(status);
    return;
  }

  // If the registered status is found, update status
  itr->stamp = stamp;
  itr->safe = safe;
  itr->start_distance = start_distance;
  itr->finish_distance = finish_distance;
  itr->auto_mode = is_auto_mode_;
}

void RTCInterface::removeCooperateStatus(const UUID & uuid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  removeStoredCommand(uuid);
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

void RTCInterface::removeStoredCommand(const UUID & uuid)
{
  // Find stored command which has same uuid and erase it
  const auto itr = std::find_if(
    stored_commands_.begin(), stored_commands_.end(), [uuid](auto & s) { return s.uuid == uuid; });

  if (itr != stored_commands_.end()) {
    stored_commands_.erase(itr);
    return;
  }
}

void RTCInterface::clearCooperateStatus()
{
  std::lock_guard<std::mutex> lock(mutex_);
  registered_status_.statuses.clear();
  stored_commands_.clear();
}

bool RTCInterface::isActivated(const UUID & uuid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });

  if (itr != registered_status_.statuses.end()) {
    if (itr->auto_mode) {
      return itr->safe;
    } else {
      return itr->command_status.type == Command::ACTIVATE;
    }
  }

  RCLCPP_WARN_STREAM(
    getLogger(), "[isActivated] uuid : " << to_string(uuid) << " is not found." << std::endl);
  return false;
}

bool RTCInterface::isRegistered(const UUID & uuid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto itr = std::find_if(
    registered_status_.statuses.begin(), registered_status_.statuses.end(),
    [uuid](auto & s) { return s.uuid == uuid; });
  return itr != registered_status_.statuses.end();
}

void RTCInterface::lockCommandUpdate() { is_locked_ = true; }

void RTCInterface::unlockCommandUpdate()
{
  is_locked_ = false;
  updateCooperateCommandStatus(stored_commands_);
}

rclcpp::Logger RTCInterface::getLogger() const { return logger_; }

bool RTCInterface::isLocked() const { return is_locked_; }

}  // namespace rtc_interface
