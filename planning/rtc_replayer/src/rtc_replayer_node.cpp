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

#include "rtc_replayer/rtc_replayer_node.hpp"

#include <algorithm>

namespace rtc_replayer
{

std::string getModuleStatus(const uint8_t module_status)
{
  switch (module_status) {
    case Command::ACTIVATE: {
      return "execute";
    }
    case Command::DEACTIVATE: {
      return "wait";
    }
  }
  return "none";
}

std::string getModuleName(const uint8_t module_type)
{
  switch (module_type) {
    case Module::LANE_CHANGE_LEFT: {
      return "lane_change_left";
    }
    case Module::LANE_CHANGE_RIGHT: {
      return "lane_change_right";
    }
    case Module::AVOIDANCE_LEFT: {
      return "avoidance_left";
    }
    case Module::AVOIDANCE_RIGHT: {
      return "avoidance_right";
    }
    case Module::GOAL_PLANNER: {
      return "goal_planner";
    }
    case Module::START_PLANNER: {
      return "start_planner";
    }
    case Module::TRAFFIC_LIGHT: {
      return "traffic_light";
    }
    case Module::INTERSECTION: {
      return "intersection";
    }
    case Module::CROSSWALK: {
      return "crosswalk";
    }
    case Module::BLIND_SPOT: {
      return "blind_spot";
    }
    case Module::DETECTION_AREA: {
      return "detection_area";
    }
    case Module::NO_STOPPING_AREA: {
      return "no_stopping_area";
    }
    case Module::OCCLUSION_SPOT: {
      return "occlusion_spot";
    }
  }
  return "NONE";
}

std::string to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}

RTCReplayerNode::RTCReplayerNode(const rclcpp::NodeOptions & node_options)
: Node("rtc_replayer_node", node_options)
{
  sub_statuses_ = create_subscription<CooperateStatusArray>(
    "/debug/rtc_status", 1, std::bind(&RTCReplayerNode::onCooperateStatus, this, _1));
  client_rtc_commands_ = create_client<CooperateCommands>("/api/external/set/rtc_commands");
}

void RTCReplayerNode::onCooperateStatus(const CooperateStatusArray::ConstSharedPtr msg)
{
  if (msg->statuses.empty()) return;
  CooperateCommands::Request::SharedPtr request = std::make_shared<CooperateCommands::Request>();
  for (auto status : msg->statuses) {
    const auto cmd_status = status.command_status.type;
    const auto uuid_string = to_string(status.uuid);
    // add command which has change from previous status and command is already registered
    if (
      prev_cmd_status_.find(uuid_string) != prev_cmd_status_.end() &&
      cmd_status != prev_cmd_status_[uuid_string]) {
      CooperateCommand cc;
      // send previous command status
      cc.command.type = cmd_status;
      cc.uuid = status.uuid;
      cc.module = status.module;
      request->stamp = status.stamp;
      request->commands.emplace_back(cc);
      std::cerr << "uuid: " << uuid_string << " module: " << getModuleName(cc.module.type)
                << " status: " << getModuleStatus(cmd_status) << std::endl;
    }
    // post process
    prev_cmd_status_[uuid_string] = cmd_status;
  }
  if (!request->commands.empty()) {
    client_rtc_commands_->async_send_request(request);
  }
}

}  // namespace rtc_replayer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rtc_replayer::RTCReplayerNode)
