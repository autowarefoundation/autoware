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

#include "rtc_auto_approver/rtc_auto_approver_interface.hpp"

namespace rtc_auto_approver
{
RTCAutoApproverInterface::RTCAutoApproverInterface(
  rclcpp::Node * node, const std::string & name, const bool default_enable)
: enabled_{default_enable}
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Subscriber
  status_sub_ = node->create_subscription<CooperateStatusArray>(
    name + "/cooperate_status", rclcpp::QoS(1),
    std::bind(&RTCAutoApproverInterface::onStatus, this, _1));

  // Service client
  command_cli_ = node->create_client<CooperateCommands>(
    name + "/cooperate_commands", rmw_qos_profile_services_default);

  // Service
  enable_srv_ = node->create_service<AutoMode>(
    name + "/enable_auto_mode",
    std::bind(&RTCAutoApproverInterface::onEnableService, this, _1, _2));
}

void RTCAutoApproverInterface::onStatus(const CooperateStatusArray::ConstSharedPtr msg) const
{
  if (!msg || !enabled_) {
    return;
  }

  const auto request = std::make_shared<CooperateCommands::Request>(createRequest(*msg));

  if (!request->commands.empty()) {
    command_cli_->async_send_request(request);
  }
}

void RTCAutoApproverInterface::onEnableService(
  const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response)
{
  enabled_ = request->enable;
  response->success = true;
}

bool RTCAutoApproverInterface::isNecessarySendCommand(const CooperateStatus & status) const
{
  const bool is_activate = (status.command_status.type == Command::ACTIVATE);
  return status.safe ^ is_activate;
}

CooperateCommands::Request RTCAutoApproverInterface::createRequest(
  const CooperateStatusArray & array) const
{
  CooperateCommands::Request request;
  request.stamp = array.stamp;

  for (const auto & status : array.statuses) {
    if (isNecessarySendCommand(status)) {
      CooperateCommand cmd;
      cmd.module = status.module;
      cmd.uuid = status.uuid;
      if (status.command_status.type == Command::DEACTIVATE) {
        cmd.command.type = Command::ACTIVATE;
        request.commands.push_back(cmd);
      } else if (status.command_status.type == Command::ACTIVATE) {
        cmd.command.type = Command::DEACTIVATE;
        request.commands.push_back(cmd);
      }
    }
  }

  return request;
}

}  // namespace rtc_auto_approver
