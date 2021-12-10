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

#ifndef EXTERNAL_CMD_CONVERTER__NODE_HPP_
#define EXTERNAL_CMD_CONVERTER__NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <raw_vehicle_cmd_converter/accel_map.hpp>
#include <raw_vehicle_cmd_converter/brake_map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_external_api_msgs/msg/control_command_stamped.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>

#include <memory>
#include <string>

namespace external_cmd_converter
{
using GearCommand = autoware_auto_vehicle_msgs::msg::GearCommand;
using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using ExternalControlCommand = tier4_external_api_msgs::msg::ControlCommandStamped;
using Odometry = nav_msgs::msg::Odometry;
using raw_vehicle_cmd_converter::AccelMap;
using raw_vehicle_cmd_converter::BrakeMap;
using ControlCommandStamped = autoware_auto_control_msgs::msg::AckermannControlCommand;
using Odometry = nav_msgs::msg::Odometry;

class ExternalCmdConverterNode : public rclcpp::Node
{
public:
  explicit ExternalCmdConverterNode(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr
    pub_current_cmd_;

  // Subscriber
  rclcpp::Subscription<Odometry>::SharedPtr sub_velocity_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::ControlCommandStamped>::SharedPtr
    sub_control_cmd_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_shift_cmd_;
  rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<tier4_external_api_msgs::msg::Heartbeat>::SharedPtr
    sub_emergency_stop_heartbeat_;

  void onVelocity(const Odometry::ConstSharedPtr msg);
  void onExternalCmd(const ExternalControlCommand::ConstSharedPtr cmd_ptr);
  void onGearCommand(const GearCommand::ConstSharedPtr msg);
  void onGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onEmergencyStopHeartbeat(const tier4_external_api_msgs::msg::Heartbeat::ConstSharedPtr msg);

  std::shared_ptr<double> current_velocity_ptr_;  // [m/s]
  std::shared_ptr<rclcpp::Time> latest_emergency_stop_heartbeat_received_time_;
  std::shared_ptr<rclcpp::Time> latest_cmd_received_time_;
  GearCommand::ConstSharedPtr current_shift_cmd_;
  tier4_control_msgs::msg::GateMode::ConstSharedPtr current_gate_mode_;

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr rate_check_timer_;

  // Parameter
  double ref_vel_gain_;  // reference velocity = current velocity + desired acceleration * gain
  bool wait_for_first_topic_;
  double control_command_timeout_;
  double emergency_stop_timeout_;

  // Diagnostics
  diagnostic_updater::Updater updater_{this};

  void checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat);
  bool checkEmergencyStopTopicTimeout();
  bool checkRemoteTopicRate();

  // Algorithm
  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;

  double calculateAcc(const ExternalControlCommand & cmd, const double vel);
  double getShiftVelocitySign(const GearCommand & cmd);
};

}  // namespace external_cmd_converter

#endif  // EXTERNAL_CMD_CONVERTER__NODE_HPP_
