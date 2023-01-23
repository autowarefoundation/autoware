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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "compatibility.hpp"
#include "state.hpp"

#include <component_interface_specs/system.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <unordered_map>
#include <utility>

namespace operation_mode_transition_manager
{

class OperationModeTransitionManager : public rclcpp::Node
{
public:
  explicit OperationModeTransitionManager(const rclcpp::NodeOptions & options);

private:
  using ChangeAutowareControlAPI = system_interface::ChangeAutowareControl;
  using ChangeOperationModeAPI = system_interface::ChangeOperationMode;
  using OperationModeStateAPI = system_interface::OperationModeState;
  component_interface_utils::Service<ChangeAutowareControlAPI>::SharedPtr srv_autoware_control_;
  component_interface_utils::Service<ChangeOperationModeAPI>::SharedPtr srv_operation_mode_;
  component_interface_utils::Publisher<OperationModeStateAPI>::SharedPtr pub_operation_mode_;
  void onChangeAutowareControl(
    const ChangeAutowareControlAPI::Service::Request::SharedPtr request,
    const ChangeAutowareControlAPI::Service::Response::SharedPtr response);
  void onChangeOperationMode(
    const ChangeOperationModeAPI::Service::Request::SharedPtr request,
    const ChangeOperationModeAPI::Service::Response::SharedPtr response);

  using ControlModeCommandType = ControlModeCommand::Request::_mode_type;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_report_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_gate_operation_mode_;
  rclcpp::Client<ControlModeCommand>::SharedPtr cli_control_mode_;
  rclcpp::Publisher<ModeChangeBase::DebugInfo>::SharedPtr pub_debug_info_;
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();
  void publishData();
  void changeControlMode(ControlModeCommandType mode);
  void changeOperationMode(std::optional<OperationMode> mode);
  void cancelTransition();
  void processTransition();

  double transition_timeout_;
  OperationMode current_mode_;
  std::unique_ptr<Transition> transition_;
  std::unordered_map<OperationMode, std::unique_ptr<ModeChangeBase>> modes_;
  std::unordered_map<OperationMode, bool> available_mode_change_;
  OperationModeState gate_operation_mode_;
  ControlModeReport control_mode_report_;

  std::optional<OperationModeStateAPI::Message> prev_state_;

  static constexpr double compatibility_timeout_ = 1.0;
  Compatibility compatibility_;
  std::optional<rclcpp::Time> compatibility_transition_;
};

}  // namespace operation_mode_transition_manager

#endif  // NODE_HPP_
