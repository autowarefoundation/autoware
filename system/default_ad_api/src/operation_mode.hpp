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

#ifndef OPERATION_MODE_HPP_
#define OPERATION_MODE_HPP_

#include <autoware_ad_api_specs/operation_mode.hpp>
#include <component_interface_specs/system.hpp>
#include <component_interface_utils/status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <unordered_map>
#include <vector>

// TODO(Takagi, Isamu): define interface
#include <tier4_system_msgs/msg/mode_change_available.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{
class OperationModeNode : public rclcpp::Node
{
public:
  explicit OperationModeNode(const rclcpp::NodeOptions & options);

private:
  using OperationModeState = autoware_ad_api::operation_mode::OperationModeState;
  using EnableAutowareControl = autoware_ad_api::operation_mode::EnableAutowareControl;
  using DisableAutowareControl = autoware_ad_api::operation_mode::DisableAutowareControl;
  using ChangeToStop = autoware_ad_api::operation_mode::ChangeToStop;
  using ChangeToAutonomous = autoware_ad_api::operation_mode::ChangeToAutonomous;
  using ChangeToLocal = autoware_ad_api::operation_mode::ChangeToLocal;
  using ChangeToRemote = autoware_ad_api::operation_mode::ChangeToRemote;
  using OperationModeRequest = system_interface::ChangeOperationMode::Service::Request;
  using AutowareControlRequest = system_interface::ChangeAutowareControl::Service::Request;
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;

  OperationModeState::Message curr_state_;
  OperationModeState::Message prev_state_;
  std::unordered_map<OperationModeState::Message::_mode_type, bool> mode_available_;

  rclcpp::CallbackGroup::SharedPtr group_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
  Pub<autoware_ad_api::operation_mode::OperationModeState> pub_state_;
  Srv<autoware_ad_api::operation_mode::ChangeToStop> srv_stop_mode_;
  Srv<autoware_ad_api::operation_mode::ChangeToAutonomous> srv_autonomous_mode_;
  Srv<autoware_ad_api::operation_mode::ChangeToLocal> srv_local_mode_;
  Srv<autoware_ad_api::operation_mode::ChangeToRemote> srv_remote_mode_;
  Srv<autoware_ad_api::operation_mode::EnableAutowareControl> srv_enable_control_;
  Srv<autoware_ad_api::operation_mode::DisableAutowareControl> srv_disable_control_;
  Sub<system_interface::OperationModeState> sub_state_;
  Cli<system_interface::ChangeOperationMode> cli_mode_;
  Cli<system_interface::ChangeAutowareControl> cli_control_;

  std::vector<bool> module_states_;
  std::vector<rclcpp::Subscription<ModeChangeAvailable>::SharedPtr> sub_module_states_;

  void on_change_to_stop(
    const ChangeToStop::Service::Request::SharedPtr req,
    const ChangeToStop::Service::Response::SharedPtr res);
  void on_change_to_autonomous(
    const ChangeToAutonomous::Service::Request::SharedPtr req,
    const ChangeToAutonomous::Service::Response::SharedPtr res);
  void on_change_to_local(
    const ChangeToLocal::Service::Request::SharedPtr req,
    const ChangeToLocal::Service::Response::SharedPtr res);
  void on_change_to_remote(
    const ChangeToRemote::Service::Request::SharedPtr req,
    const ChangeToRemote::Service::Response::SharedPtr res);
  void on_enable_autoware_control(
    const EnableAutowareControl::Service::Request::SharedPtr req,
    const EnableAutowareControl::Service::Response::SharedPtr res);
  void on_disable_autoware_control(
    const DisableAutowareControl::Service::Request::SharedPtr req,
    const DisableAutowareControl::Service::Response::SharedPtr res);

  void on_state(const OperationModeState::Message::ConstSharedPtr msg);
  void on_timer();
  void update_state();

  template <class ResponseT>
  void change_mode(const ResponseT res, const OperationModeRequest::_mode_type mode);
};

}  // namespace default_ad_api

#endif  // OPERATION_MODE_HPP_
