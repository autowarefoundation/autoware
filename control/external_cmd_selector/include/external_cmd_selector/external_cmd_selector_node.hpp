// Copyright 2020 Tier IV, Inc.
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

#ifndef EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
#define EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_control_msgs/msg/external_command_selector_mode.hpp>
#include <tier4_control_msgs/srv/external_command_select.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/gear_shift_stamped.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>
#include <tier4_external_api_msgs/msg/turn_signal_stamped.hpp>

#include <memory>

class ExternalCmdSelector : public rclcpp::Node
{
public:
  explicit ExternalCmdSelector(const rclcpp::NodeOptions & node_options);

private:
  using CommandSourceSelect = tier4_control_msgs::srv::ExternalCommandSelect;
  using CommandSourceMode = tier4_control_msgs::msg::ExternalCommandSelectorMode;
  using InternalGearShift = autoware_auto_vehicle_msgs::msg::GearCommand;
  using InternalTurnSignal = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
  using InternalHazardSignal = autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
  using InternalHeartbeat = tier4_external_api_msgs::msg::Heartbeat;
  using ExternalControlCommand = tier4_external_api_msgs::msg::ControlCommandStamped;
  using ExternalGearShift = tier4_external_api_msgs::msg::GearShiftStamped;
  using ExternalTurnSignal = tier4_external_api_msgs::msg::TurnSignalStamped;
  using ExternalHeartbeat = tier4_external_api_msgs::msg::Heartbeat;

  // CallbackGroups
  rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_services_;

  // Publisher
  rclcpp::Publisher<CommandSourceMode>::SharedPtr pub_current_selector_mode_;
  rclcpp::Publisher<ExternalControlCommand>::SharedPtr pub_control_cmd_;
  rclcpp::Publisher<InternalGearShift>::SharedPtr pub_shift_cmd_;
  rclcpp::Publisher<InternalTurnSignal>::SharedPtr pub_turn_signal_cmd_;
  rclcpp::Publisher<InternalHazardSignal>::SharedPtr pub_hazard_signal_cmd_;
  rclcpp::Publisher<InternalHeartbeat>::SharedPtr pub_heartbeat_;

  // Subscriber
  rclcpp::Subscription<ExternalControlCommand>::SharedPtr sub_local_control_cmd_;
  rclcpp::Subscription<ExternalGearShift>::SharedPtr sub_local_shift_cmd_;
  rclcpp::Subscription<ExternalTurnSignal>::SharedPtr sub_local_turn_signal_cmd_;
  rclcpp::Subscription<ExternalHeartbeat>::SharedPtr sub_local_heartbeat_;

  rclcpp::Subscription<ExternalControlCommand>::SharedPtr sub_remote_control_cmd_;
  rclcpp::Subscription<ExternalGearShift>::SharedPtr sub_remote_shift_cmd_;
  rclcpp::Subscription<ExternalTurnSignal>::SharedPtr sub_remote_turn_signal_cmd_;
  rclcpp::Subscription<ExternalHeartbeat>::SharedPtr sub_remote_heartbeat_;

  void onLocalControlCmd(const ExternalControlCommand::ConstSharedPtr msg);
  void onLocalShiftCmd(const ExternalGearShift::ConstSharedPtr msg);
  void onLocalTurnSignalCmd(const ExternalTurnSignal::ConstSharedPtr msg);
  void onLocalHeartbeat(const ExternalHeartbeat::ConstSharedPtr msg);

  void onRemoteControlCmd(const ExternalControlCommand::ConstSharedPtr msg);
  void onRemoteShiftCmd(const ExternalGearShift::ConstSharedPtr msg);
  void onRemoteTurnSignalCmd(const ExternalTurnSignal::ConstSharedPtr msg);
  void onRemoteHeartbeat(const ExternalHeartbeat::ConstSharedPtr msg);

  // Service
  rclcpp::Service<CommandSourceSelect>::SharedPtr srv_select_external_command_;
  CommandSourceMode current_selector_mode_;

  bool onSelectExternalCommandService(
    const CommandSourceSelect::Request::SharedPtr req,
    const CommandSourceSelect::Response::SharedPtr res);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();

  // Converter
  static InternalGearShift convert(const ExternalGearShift & command);
  static InternalHeartbeat convert(const ExternalHeartbeat & command);

  // Diagnostics Updater
  diagnostic_updater::Updater updater_{this};
};

#endif  // EXTERNAL_CMD_SELECTOR__EXTERNAL_CMD_SELECTOR_NODE_HPP_
