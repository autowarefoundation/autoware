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

#include "external_cmd_selector/external_cmd_selector_node.hpp"

#include <tier4_auto_msgs_converter/tier4_auto_msgs_converter.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

ExternalCmdSelector::ExternalCmdSelector(const rclcpp::NodeOptions & node_options)
: Node("external_cmd_selector", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Parameter
  double update_rate = declare_parameter<double>("update_rate");
  std::string initial_selector_mode = declare_parameter<std::string>("initial_selector_mode");

  // Publisher
  pub_current_selector_mode_ =
    create_publisher<CommandSourceMode>("~/output/current_selector_mode", 1);
  pub_control_cmd_ = create_publisher<ExternalControlCommand>("~/output/control_cmd", 1);
  pub_shift_cmd_ = create_publisher<InternalGearShift>("~/output/gear_cmd", 1);
  pub_turn_signal_cmd_ = create_publisher<InternalTurnSignal>("~/output/turn_indicators_cmd", 1);
  pub_hazard_signal_cmd_ = create_publisher<InternalHazardSignal>("~/output/hazard_lights_cmd", 1);
  pub_heartbeat_ = create_publisher<InternalHeartbeat>("~/output/heartbeat", 1);

  // Callback Groups
  callback_group_subscribers_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  // Subscriber
  sub_local_control_cmd_ = create_subscription<ExternalControlCommand>(
    "~/input/local/control_cmd", 1, std::bind(&ExternalCmdSelector::onLocalControlCmd, this, _1),
    subscriber_option);
  sub_local_shift_cmd_ = create_subscription<ExternalGearShift>(
    "~/input/local/shift_cmd", 1, std::bind(&ExternalCmdSelector::onLocalShiftCmd, this, _1),
    subscriber_option);
  sub_local_turn_signal_cmd_ = create_subscription<ExternalTurnSignal>(
    "~/input/local/turn_signal_cmd", 1,
    std::bind(&ExternalCmdSelector::onLocalTurnSignalCmd, this, _1), subscriber_option);
  sub_local_heartbeat_ = create_subscription<ExternalHeartbeat>(
    "~/input/local/heartbeat", 1, std::bind(&ExternalCmdSelector::onLocalHeartbeat, this, _1),
    subscriber_option);

  sub_remote_control_cmd_ = create_subscription<ExternalControlCommand>(
    "~/input/remote/control_cmd", 1, std::bind(&ExternalCmdSelector::onRemoteControlCmd, this, _1),
    subscriber_option);
  sub_remote_shift_cmd_ = create_subscription<ExternalGearShift>(
    "~/input/remote/shift_cmd", 1, std::bind(&ExternalCmdSelector::onRemoteShiftCmd, this, _1),
    subscriber_option);
  sub_remote_turn_signal_cmd_ = create_subscription<ExternalTurnSignal>(
    "~/input/remote/turn_signal_cmd", 1,
    std::bind(&ExternalCmdSelector::onRemoteTurnSignalCmd, this, _1), subscriber_option);
  sub_remote_heartbeat_ = create_subscription<ExternalHeartbeat>(
    "~/input/remote/heartbeat", 1, std::bind(&ExternalCmdSelector::onRemoteHeartbeat, this, _1),
    subscriber_option);

  // Service
  srv_select_external_command_ = create_service<CommandSourceSelect>(
    "~/service/select_external_command",
    std::bind(&ExternalCmdSelector::onSelectExternalCommandService, this, _1, _2),
    rmw_qos_profile_services_default, callback_group_services_);

  // Initialize mode
  auto convert_selector_mode = [](const std::string & mode_text) {
    if (mode_text == "local") {
      return CommandSourceMode::LOCAL;
    }
    if (mode_text == "remote") {
      return CommandSourceMode::REMOTE;
    }
    throw std::invalid_argument("unknown selector mode");
  };
  current_selector_mode_.data = convert_selector_mode(initial_selector_mode);

  // Diagnostics Updater
  updater_.setHardwareID("external_cmd_selector");
  updater_.add("heartbeat", [](auto & stat) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Alive");
  });

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ExternalCmdSelector::onTimer, this),
    callback_group_subscribers_);
}

void ExternalCmdSelector::onLocalControlCmd(const ExternalControlCommand::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != CommandSourceMode::LOCAL) {
    return;
  }
  pub_control_cmd_->publish(*msg);
}

void ExternalCmdSelector::onLocalShiftCmd(const ExternalGearShift::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != CommandSourceMode::LOCAL) {
    return;
  }
  pub_shift_cmd_->publish(convert(*msg));
}

void ExternalCmdSelector::onLocalTurnSignalCmd(const ExternalTurnSignal::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != CommandSourceMode::LOCAL) {
    return;
  }
  auto light_signal = tier4_auto_msgs_converter::convert(*msg);
  pub_turn_signal_cmd_->publish(light_signal.turn_signal);
  pub_hazard_signal_cmd_->publish(light_signal.hazard_signal);
}

void ExternalCmdSelector::onLocalHeartbeat(const ExternalHeartbeat::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != CommandSourceMode::LOCAL) {
    return;
  }
  pub_heartbeat_->publish(convert(*msg));
}

void ExternalCmdSelector::onRemoteControlCmd(const ExternalControlCommand::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != CommandSourceMode::REMOTE) {
    return;
  }
  pub_control_cmd_->publish(*msg);
}

void ExternalCmdSelector::onRemoteShiftCmd(const ExternalGearShift::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != CommandSourceMode::REMOTE) {
    return;
  }
  pub_shift_cmd_->publish(convert(*msg));
}

void ExternalCmdSelector::onRemoteTurnSignalCmd(const ExternalTurnSignal::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != CommandSourceMode::REMOTE) {
    return;
  }
  auto light_signal = tier4_auto_msgs_converter::convert(*msg);
  pub_turn_signal_cmd_->publish(light_signal.turn_signal);
  pub_hazard_signal_cmd_->publish(light_signal.hazard_signal);
}

void ExternalCmdSelector::onRemoteHeartbeat(const ExternalHeartbeat::ConstSharedPtr msg)
{
  if (current_selector_mode_.data != CommandSourceMode::REMOTE) {
    return;
  }
  pub_heartbeat_->publish(convert(*msg));
}

bool ExternalCmdSelector::onSelectExternalCommandService(
  const CommandSourceSelect::Request::SharedPtr req,
  const CommandSourceSelect::Response::SharedPtr res)
{
  current_selector_mode_.data = req->mode.data;
  res->success = true;
  res->message = "Success.";
  return true;
}

void ExternalCmdSelector::onTimer()
{
  pub_current_selector_mode_->publish(current_selector_mode_);
  updater_.force_update();
}

ExternalCmdSelector::InternalGearShift ExternalCmdSelector::convert(
  const ExternalGearShift & command)
{
  return tier4_auto_msgs_converter::convert(command);
}

ExternalCmdSelector::InternalHeartbeat ExternalCmdSelector::convert(
  const ExternalHeartbeat & command)
{
  return command;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ExternalCmdSelector)
