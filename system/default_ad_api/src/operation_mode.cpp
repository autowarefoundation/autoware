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

#include "operation_mode.hpp"

#include <memory>
#include <string>
#include <vector>

namespace default_ad_api
{

using ServiceResponse = autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response;

OperationModeNode::OperationModeNode(const rclcpp::NodeOptions & options)
: Node("operation_mode", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_sub(sub_state_, this, &OperationModeNode::on_state);
  adaptor.init_pub(pub_state_);
  adaptor.init_srv(srv_stop_mode_, this, &OperationModeNode::on_change_to_stop);
  adaptor.init_srv(srv_autonomous_mode_, this, &OperationModeNode::on_change_to_autonomous);
  adaptor.init_srv(srv_local_mode_, this, &OperationModeNode::on_change_to_local);
  adaptor.init_srv(srv_remote_mode_, this, &OperationModeNode::on_change_to_remote);
  adaptor.init_srv(srv_enable_control_, this, &OperationModeNode::on_enable_autoware_control);
  adaptor.init_srv(srv_disable_control_, this, &OperationModeNode::on_disable_autoware_control);
  adaptor.init_cli(cli_mode_, group_cli_);
  adaptor.init_cli(cli_control_, group_cli_);

  const std::vector<std::string> module_names = {
    "sensing", "perception", "map", "localization", "planning", "control", "vehicle", "system",
  };

  for (size_t i = 0; i < module_names.size(); ++i) {
    const auto name = "/system/component_state_monitor/component/autonomous/" + module_names[i];
    const auto qos = rclcpp::QoS(1).transient_local();
    const auto callback = [this, i, module_names](const ModeChangeAvailable::ConstSharedPtr msg) {
      module_states_[module_names[i]] = msg->available;
    };
    sub_module_states_.push_back(create_subscription<ModeChangeAvailable>(name, qos, callback));
  }

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(5.0).period(), std::bind(&OperationModeNode::on_timer, this));

  curr_state_.mode = OperationModeState::Message::UNKNOWN;
  prev_state_.mode = OperationModeState::Message::UNKNOWN;
  mode_available_[OperationModeState::Message::UNKNOWN] = false;
  mode_available_[OperationModeState::Message::STOP] = true;
  mode_available_[OperationModeState::Message::AUTONOMOUS] = false;
  mode_available_[OperationModeState::Message::LOCAL] = true;
  mode_available_[OperationModeState::Message::REMOTE] = true;
}

template <class ResponseT>
void OperationModeNode::change_mode(
  const ResponseT res, const OperationModeRequest::_mode_type mode)
{
  if (!mode_available_[mode]) {
    throw component_interface_utils::ServiceException(
      ServiceResponse::ERROR_NOT_AVAILABLE, "The mode change is blocked by the system.");
  }
  const auto req = std::make_shared<OperationModeRequest>();
  req->mode = mode;
  component_interface_utils::status::copy(cli_mode_->call(req), res);  // NOLINT
}

void OperationModeNode::on_change_to_stop(
  const ChangeToStop::Service::Request::SharedPtr,
  const ChangeToStop::Service::Response::SharedPtr res)
{
  change_mode(res, OperationModeRequest::STOP);
}

void OperationModeNode::on_change_to_autonomous(
  const ChangeToAutonomous::Service::Request::SharedPtr,
  const ChangeToAutonomous::Service::Response::SharedPtr res)
{
  change_mode(res, OperationModeRequest::AUTONOMOUS);
}

void OperationModeNode::on_change_to_local(
  const ChangeToLocal::Service::Request::SharedPtr,
  const ChangeToLocal::Service::Response::SharedPtr res)
{
  change_mode(res, OperationModeRequest::LOCAL);
}

void OperationModeNode::on_change_to_remote(
  const ChangeToRemote::Service::Request::SharedPtr,
  const ChangeToRemote::Service::Response::SharedPtr res)
{
  change_mode(res, OperationModeRequest::REMOTE);
}

void OperationModeNode::on_enable_autoware_control(
  const EnableAutowareControl::Service::Request::SharedPtr,
  const EnableAutowareControl::Service::Response::SharedPtr res)
{
  if (!mode_available_[curr_state_.mode]) {
    throw component_interface_utils::ServiceException(
      ServiceResponse::ERROR_NOT_AVAILABLE, "The mode change is blocked by the system.");
  }
  const auto req = std::make_shared<AutowareControlRequest>();
  req->autoware_control = true;
  component_interface_utils::status::copy(cli_control_->call(req), res);  // NOLINT
}

void OperationModeNode::on_disable_autoware_control(
  const DisableAutowareControl::Service::Request::SharedPtr,
  const DisableAutowareControl::Service::Response::SharedPtr res)
{
  const auto req = std::make_shared<AutowareControlRequest>();
  req->autoware_control = false;
  component_interface_utils::status::copy(cli_control_->call(req), res);  // NOLINT
}

void OperationModeNode::on_state(const OperationModeState::Message::ConstSharedPtr msg)
{
  curr_state_ = *msg;
  update_state();
}

void OperationModeNode::on_timer()
{
  bool autonomous_available = true;
  std::string unhealthy_components = "";
  for (const auto & state : module_states_) {
    if (!state.second) {
      unhealthy_components += unhealthy_components.empty() ? state.first : ", " + state.first;
    }
    autonomous_available &= state.second;
  }
  mode_available_[OperationModeState::Message::AUTONOMOUS] = autonomous_available;

  if (!unhealthy_components.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "%s component state is unhealthy. Autonomous is not available.",
      unhealthy_components.c_str());
  }

  update_state();
}

void OperationModeNode::update_state()
{
  // Clear stamp to compare other fields.
  OperationModeState::Message state = curr_state_;
  state.stamp = builtin_interfaces::msg::Time();
  state.is_stop_mode_available &= mode_available_[OperationModeState::Message::STOP];
  state.is_autonomous_mode_available &= mode_available_[OperationModeState::Message::AUTONOMOUS];
  state.is_local_mode_available &= mode_available_[OperationModeState::Message::LOCAL];
  state.is_remote_mode_available &= mode_available_[OperationModeState::Message::REMOTE];

  if (prev_state_ != state) {
    prev_state_ = state;
    state.stamp = now();
    pub_state_->publish(state);
  }
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::OperationModeNode)
