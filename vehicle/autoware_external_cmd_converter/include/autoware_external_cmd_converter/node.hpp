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

#ifndef AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
#define AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_

#include "autoware/universe_utils/ros/polling_subscriber.hpp"

#include <autoware_raw_vehicle_cmd_converter/accel_map.hpp>
#include <autoware_raw_vehicle_cmd_converter/brake_map.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>

#include <memory>
#include <string>

namespace autoware::external_cmd_converter
{
using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
using autoware_control_msgs::msg::Control;
using ExternalControlCommand = tier4_external_api_msgs::msg::ControlCommandStamped;
using autoware::raw_vehicle_cmd_converter::AccelMap;
using autoware::raw_vehicle_cmd_converter::BrakeMap;
using nav_msgs::msg::Odometry;
using tier4_control_msgs::msg::GateMode;

class ExternalCmdConverterNode : public rclcpp::Node
{
public:
  explicit ExternalCmdConverterNode(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  rclcpp::Publisher<Control>::SharedPtr cmd_pub_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr
    current_cmd_pub_;

  // Subscriber
  rclcpp::Subscription<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::Heartbeat>::SharedPtr
    emergency_stop_heartbeat_sub_;

  // Polling Subscriber
  autoware_universe_utils::InterProcessPollingSubscriber<Odometry> velocity_sub_{
    this, "in/odometry"};
  autoware_universe_utils::InterProcessPollingSubscriber<GearCommand> shift_cmd_sub_{
    this, "in/shift_cmd"};
  autoware_universe_utils::InterProcessPollingSubscriber<GateMode> gate_mode_sub_{
    this, "in/current_gate_mode"};

  void on_external_cmd(const ExternalControlCommand::ConstSharedPtr cmd_ptr);
  void on_emergency_stop_heartbeat(
    const tier4_external_api_msgs::msg::Heartbeat::ConstSharedPtr msg);

  Odometry::ConstSharedPtr current_velocity_ptr_{nullptr};  // [m/s]
  GearCommand::ConstSharedPtr current_shift_cmd_{nullptr};
  GateMode::ConstSharedPtr current_gate_mode_{nullptr};

  std::shared_ptr<rclcpp::Time> latest_emergency_stop_heartbeat_received_time_;
  std::shared_ptr<rclcpp::Time> latest_cmd_received_time_;

  // Timer
  void on_timer();
  rclcpp::TimerBase::SharedPtr rate_check_timer_;

  // Parameter
  double ref_vel_gain_;  // reference velocity = current velocity + desired acceleration * gain
  bool wait_for_first_topic_;
  double control_command_timeout_;
  double emergency_stop_timeout_;

  // Diagnostics
  diagnostic_updater::Updater updater_{this};

  void check_topic_status(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void check_emergency_stop(diagnostic_updater::DiagnosticStatusWrapper & stat);
  bool check_emergency_stop_topic_timeout();
  bool check_remote_topic_rate();

  // Algorithm
  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;

  double calculate_acc(const ExternalControlCommand & cmd, const double vel);
  double get_shift_velocity_sign(const GearCommand & cmd);
};

}  // namespace autoware::external_cmd_converter

#endif  // AUTOWARE_EXTERNAL_CMD_CONVERTER__NODE_HPP_
