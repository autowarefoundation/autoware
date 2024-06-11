// Copyright 2022 Autoware Foundation
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

#include "compatibility.hpp"

#include <memory>

namespace autoware::operation_mode_transition_manager
{

Compatibility::Compatibility(rclcpp::Node * node) : node_(node)
{
  sub_autoware_engage_ = node->create_subscription<AutowareEngage>(
    "/api/autoware/get/engage", 1,
    std::bind(&Compatibility::on_autoware_engage, this, std::placeholders::_1));
  sub_gate_mode_ = node->create_subscription<GateMode>(
    "/control/current_gate_mode", 1,
    std::bind(&Compatibility::on_gate_mode, this, std::placeholders::_1));
  sub_selector_mode_ = node->create_subscription<SelectorModeMsg>(
    "/control/external_cmd_selector/current_selector_mode", 1,
    std::bind(&Compatibility::on_selector_mode, this, std::placeholders::_1));

  pub_autoware_engage_ = node->create_publisher<AutowareEngage>("/autoware/engage", 1);
  pub_gate_mode_ = node->create_publisher<GateMode>("/control/gate_mode_cmd", 1);
  cli_selector_mode_ =
    node->create_client<SelectorModeSrv>("/control/external_cmd_selector/select_external_command");
}

void Compatibility::on_autoware_engage(const AutowareEngage::ConstSharedPtr msg)
{
  autoware_engage_ = msg;
}

void Compatibility::on_gate_mode(const GateMode::ConstSharedPtr msg)
{
  gate_mode_ = msg;
}

void Compatibility::on_selector_mode(const SelectorModeMsg::ConstSharedPtr msg)
{
  selector_mode_ = msg;
}

std::optional<OperationMode> Compatibility::get_mode() const
{
  if (!(autoware_engage_ && gate_mode_ && selector_mode_)) {
    return std::nullopt;
  }

  if (autoware_engage_->engage == false) {
    return OperationMode::STOP;
  }
  if (gate_mode_->data == GateMode::AUTO) {
    return OperationMode::AUTONOMOUS;
  }
  if (selector_mode_->data == SelectorModeMsg::REMOTE) {
    return OperationMode::REMOTE;
  }
  if (selector_mode_->data == SelectorModeMsg::LOCAL) {
    return OperationMode::LOCAL;
  }
  return std::nullopt;
}

void Compatibility::set_mode(const OperationMode mode)
{
  // Set operation mode in order from upstream node
  if (!(autoware_engage_ && gate_mode_ && selector_mode_)) {
    return;
  }

  // Convert mode for each node.
  AutowareEngage::_engage_type autoware_engage;
  GateMode::_data_type gate_mode;
  SelectorModeMsg::_data_type selector_mode;
  switch (mode) {
    case OperationMode::STOP:
      autoware_engage = false;
      gate_mode = GateMode::AUTO;
      selector_mode = SelectorModeMsg::NONE;
      break;
    case OperationMode::AUTONOMOUS:
      autoware_engage = true;
      gate_mode = GateMode::AUTO;
      selector_mode = SelectorModeMsg::NONE;
      break;
    case OperationMode::LOCAL:
      autoware_engage = true;
      gate_mode = GateMode::EXTERNAL;
      selector_mode = SelectorModeMsg::LOCAL;
      break;
    case OperationMode::REMOTE:
      autoware_engage = true;
      gate_mode = GateMode::EXTERNAL;
      selector_mode = SelectorModeMsg::REMOTE;
      break;
    default:
      RCLCPP_ERROR_STREAM(node_->get_logger(), "unknown mode");
      return;
  }

  // Set selector mode.
  if (selector_mode != SelectorModeMsg::NONE && selector_mode_->data != selector_mode) {
    if (!is_calling_service_) {
      auto req = std::make_shared<SelectorModeSrv::Request>();
      req->mode.data = selector_mode;
      is_calling_service_ = true;
      cli_selector_mode_->async_send_request(
        req,
        [this](rclcpp::Client<SelectorModeSrv>::SharedFuture) { is_calling_service_ = false; });
    }
  }

  // Set gate mode.
  if (gate_mode_->data != gate_mode) {
    GateMode msg;
    msg.data = gate_mode;
    pub_gate_mode_->publish(msg);
    return;
  }

  // Set autoware engage.
  if (autoware_engage_->engage != autoware_engage) {
    AutowareEngage msg;
    msg.stamp = node_->now();
    msg.engage = autoware_engage;
    pub_autoware_engage_->publish(msg);
    return;
  }
}

}  // namespace autoware::operation_mode_transition_manager
