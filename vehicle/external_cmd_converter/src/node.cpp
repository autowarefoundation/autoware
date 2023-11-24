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

#include "external_cmd_converter/node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

namespace external_cmd_converter
{
ExternalCmdConverterNode::ExternalCmdConverterNode(const rclcpp::NodeOptions & node_options)
: Node("external_cmd_converter", node_options)
{
  using std::placeholders::_1;

  pub_cmd_ = create_publisher<AckermannControlCommand>("out/control_cmd", rclcpp::QoS{1});
  pub_current_cmd_ =
    create_publisher<ExternalControlCommand>("out/latest_external_control_cmd", rclcpp::QoS{1});
  sub_velocity_ = create_subscription<Odometry>(
    "in/odometry", 1, std::bind(&ExternalCmdConverterNode::onVelocity, this, _1));
  sub_control_cmd_ = create_subscription<ExternalControlCommand>(
    "in/external_control_cmd", 1, std::bind(&ExternalCmdConverterNode::onExternalCmd, this, _1));
  sub_shift_cmd_ = create_subscription<GearCommand>(
    "in/shift_cmd", 1, std::bind(&ExternalCmdConverterNode::onGearCommand, this, _1));
  sub_gate_mode_ = create_subscription<tier4_control_msgs::msg::GateMode>(
    "in/current_gate_mode", 1, std::bind(&ExternalCmdConverterNode::onGateMode, this, _1));
  sub_emergency_stop_heartbeat_ = create_subscription<tier4_external_api_msgs::msg::Heartbeat>(
    "in/emergency_stop", 1,
    std::bind(&ExternalCmdConverterNode::onEmergencyStopHeartbeat, this, _1));

  // Parameter
  ref_vel_gain_ = declare_parameter<double>("ref_vel_gain");

  // Parameter for Hz check
  const double timer_rate = declare_parameter<double>("timer_rate");
  wait_for_first_topic_ = declare_parameter<bool>("wait_for_first_topic");
  control_command_timeout_ = declare_parameter<double>("control_command_timeout");
  emergency_stop_timeout_ = declare_parameter<double>("emergency_stop_timeout");

  const auto period_ns = rclcpp::Rate(timer_rate).period();
  rate_check_timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ExternalCmdConverterNode::onTimer, this));

  // Parameter for accel/brake map
  const std::string csv_path_accel_map = declare_parameter<std::string>("csv_path_accel_map");
  const std::string csv_path_brake_map = declare_parameter<std::string>("csv_path_brake_map");
  acc_map_initialized_ = true;
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
    RCLCPP_ERROR(
      get_logger(), "Cannot read accelmap. csv path = %s. stop calculation.",
      csv_path_accel_map.c_str());
    acc_map_initialized_ = false;
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
    RCLCPP_ERROR(
      get_logger(), "Cannot read brakemap. csv path = %s. stop calculation.",
      csv_path_brake_map.c_str());
    acc_map_initialized_ = false;
  }

  // Diagnostics
  updater_.setHardwareID("external_cmd_converter");
  updater_.add("remote_control_topic_status", this, &ExternalCmdConverterNode::checkTopicStatus);

  // Set default values
  current_shift_cmd_ = std::make_shared<GearCommand>();
}

void ExternalCmdConverterNode::onTimer()
{
  updater_.force_update();
}

void ExternalCmdConverterNode::onVelocity(const Odometry::ConstSharedPtr msg)
{
  current_velocity_ptr_ = std::make_shared<double>(msg->twist.twist.linear.x);
}

void ExternalCmdConverterNode::onGearCommand(const GearCommand::ConstSharedPtr msg)
{
  current_shift_cmd_ = msg;
}

void ExternalCmdConverterNode::onEmergencyStopHeartbeat(
  [[maybe_unused]] const tier4_external_api_msgs::msg::Heartbeat::ConstSharedPtr msg)
{
  latest_emergency_stop_heartbeat_received_time_ = std::make_shared<rclcpp::Time>(this->now());
  updater_.force_update();
}

void ExternalCmdConverterNode::onExternalCmd(const ExternalControlCommand::ConstSharedPtr cmd_ptr)
{
  // Echo back received command
  {
    auto current_cmd = *cmd_ptr;
    current_cmd.stamp = this->now();
    pub_current_cmd_->publish(current_cmd);
  }

  // Save received time for rate check
  latest_cmd_received_time_ = std::make_shared<rclcpp::Time>(this->now());

  // Wait for input data
  if (!current_velocity_ptr_ || !acc_map_initialized_) {
    return;
  }

  // Calculate reference velocity and acceleration
  const double sign = getShiftVelocitySign(*current_shift_cmd_);
  const double ref_acceleration = calculateAcc(*cmd_ptr, std::fabs(*current_velocity_ptr_));

  if (ref_acceleration > 0.0 && sign == 0.0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "Target acceleration is positive, but the gear is not appropriate. accel: %f, gear: %d",
      ref_acceleration, current_shift_cmd_->command);
  }

  double ref_velocity = *current_velocity_ptr_ + ref_acceleration * ref_vel_gain_ * sign;
  if (current_shift_cmd_->command == GearCommand::REVERSE) {
    ref_velocity = std::min(0.0, ref_velocity);
  } else if (
    current_shift_cmd_->command == GearCommand::DRIVE ||  // NOLINT
    current_shift_cmd_->command == GearCommand::LOW) {
    ref_velocity = std::max(0.0, ref_velocity);
  } else {
    ref_velocity = 0.0;
    // TODO(Hiroki OTA): ref_acceleration also must be correct value for stopping.
  }

  // Publish ControlCommand
  autoware_auto_control_msgs::msg::AckermannControlCommand output;
  output.stamp = cmd_ptr->stamp;
  output.lateral.steering_tire_angle = cmd_ptr->control.steering_angle;
  output.lateral.steering_tire_rotation_rate = cmd_ptr->control.steering_angle_velocity;
  output.longitudinal.speed = ref_velocity;
  output.longitudinal.acceleration = ref_acceleration;

  pub_cmd_->publish(output);
}

double ExternalCmdConverterNode::calculateAcc(const ExternalControlCommand & cmd, const double vel)
{
  const double desired_throttle = cmd.control.throttle;
  const double desired_brake = cmd.control.brake;
  const double desired_pedal = desired_throttle - desired_brake;

  double ref_acceleration = 0.0;
  if (desired_pedal > 0.0) {
    accel_map_.getAcceleration(desired_pedal, vel, ref_acceleration);
  } else {
    brake_map_.getAcceleration(-desired_pedal, vel, ref_acceleration);
  }

  return ref_acceleration;
}

double ExternalCmdConverterNode::getShiftVelocitySign(const GearCommand & cmd)
{
  if (cmd.command == GearCommand::DRIVE) {
    return 1.0;
  }
  if (cmd.command == GearCommand::LOW) {
    return 1.0;
  }
  if (cmd.command == GearCommand::REVERSE) {
    return -1.0;
  }

  return 0.0;
}

void ExternalCmdConverterNode::checkTopicStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  DiagnosticStatus status;
  if (!checkEmergencyStopTopicTimeout()) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "emergency stop topic is timeout";
  } else if (!checkRemoteTopicRate()) {
    status.level = DiagnosticStatus::ERROR;
    status.message = "low topic rate for remote vehicle_cmd";
  } else {
    status.level = DiagnosticStatus::OK;
    status.message = "OK";
  }

  stat.summary(status.level, status.message);
}

void ExternalCmdConverterNode::onGateMode(
  const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg)
{
  current_gate_mode_ = msg;
}

bool ExternalCmdConverterNode::checkEmergencyStopTopicTimeout()
{
  if (!latest_emergency_stop_heartbeat_received_time_) {
    if (wait_for_first_topic_) {
      return true;
    } else {
      return false;
    }
  }

  const auto duration = (this->now() - *latest_emergency_stop_heartbeat_received_time_);
  if (duration.seconds() > emergency_stop_timeout_) {
    return false;
  }

  return true;
}

bool ExternalCmdConverterNode::checkRemoteTopicRate()
{
  if (!current_gate_mode_) {
    return true;
  }

  if (!latest_cmd_received_time_) {
    if (wait_for_first_topic_) {
      return true;
    } else {
      return false;
    }
  }

  if (current_gate_mode_->data == tier4_control_msgs::msg::GateMode::EXTERNAL) {
    const auto duration = (this->now() - *latest_cmd_received_time_);
    if (duration.seconds() > control_command_timeout_) {
      return false;
    }
  } else {
    latest_cmd_received_time_ = nullptr;  // reset;
  }

  return true;
}
}  // namespace external_cmd_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(external_cmd_converter::ExternalCmdConverterNode)
