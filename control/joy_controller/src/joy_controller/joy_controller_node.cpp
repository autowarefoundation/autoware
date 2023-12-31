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

#include "joy_controller/joy_controller.hpp"
#include "joy_controller/joy_converter/ds4_joy_converter.hpp"
#include "joy_controller/joy_converter/g29_joy_converter.hpp"
#include "joy_controller/joy_converter/p65_joy_converter.hpp"
#include "joy_controller/joy_converter/xbox_joy_converter.hpp"

#include <tier4_api_utils/tier4_api_utils.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

namespace
{
using joy_controller::GateModeType;
using joy_controller::GearShiftType;
using joy_controller::TurnSignalType;
using GearShift = tier4_external_api_msgs::msg::GearShift;
using TurnSignal = tier4_external_api_msgs::msg::TurnSignal;
using GateMode = tier4_control_msgs::msg::GateMode;

GearShiftType getUpperShift(const GearShiftType & shift)
{
  if (shift == GearShift::NONE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::PARKING) {
    return GearShift::REVERSE;
  }
  if (shift == GearShift::REVERSE) {
    return GearShift::NEUTRAL;
  }
  if (shift == GearShift::NEUTRAL) {
    return GearShift::DRIVE;
  }
  if (shift == GearShift::DRIVE) {
    return GearShift::LOW;
  }
  if (shift == GearShift::LOW) {
    return GearShift::LOW;
  }

  return GearShift::NONE;
}

GearShiftType getLowerShift(const GearShiftType & shift)
{
  if (shift == GearShift::NONE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::PARKING) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::REVERSE) {
    return GearShift::PARKING;
  }
  if (shift == GearShift::NEUTRAL) {
    return GearShift::REVERSE;
  }
  if (shift == GearShift::DRIVE) {
    return GearShift::NEUTRAL;
  }
  if (shift == GearShift::LOW) {
    return GearShift::DRIVE;
  }

  return GearShift::NONE;
}

const char * getShiftName(const GearShiftType & shift)
{
  if (shift == GearShift::NONE) {
    return "NONE";
  }
  if (shift == GearShift::PARKING) {
    return "PARKING";
  }
  if (shift == GearShift::REVERSE) {
    return "REVERSE";
  }
  if (shift == GearShift::NEUTRAL) {
    return "NEUTRAL";
  }
  if (shift == GearShift::DRIVE) {
    return "DRIVE";
  }
  if (shift == GearShift::LOW) {
    return "LOW";
  }

  return "NOT_SUPPORTED";
}

const char * getTurnSignalName(const TurnSignalType & turn_signal)
{
  if (turn_signal == TurnSignal::NONE) {
    return "NONE";
  }
  if (turn_signal == TurnSignal::LEFT) {
    return "LEFT";
  }
  if (turn_signal == TurnSignal::RIGHT) {
    return "RIGHT";
  }
  if (turn_signal == TurnSignal::HAZARD) {
    return "HAZARD";
  }

  return "NOT_SUPPORTED";
}

const char * getGateModeName(const GateModeType & gate_mode)
{
  using tier4_control_msgs::msg::GateMode;

  if (gate_mode == GateMode::AUTO) {
    return "AUTO";
  }
  if (gate_mode == GateMode::EXTERNAL) {
    return "EXTERNAL";
  }

  return "NOT_SUPPORTED";
}

double calcMapping(const double input, const double sensitivity)
{
  const double exponent = 1.0 / (std::max(0.001, std::min(1.0, sensitivity)));
  return std::pow(input, exponent);
}

}  // namespace

namespace joy_controller
{
void AutowareJoyControllerNode::onJoy(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  last_joy_received_time_ = msg->header.stamp;
  if (joy_type_ == "G29") {
    joy_ = std::make_shared<const G29JoyConverter>(*msg);
  } else if (joy_type_ == "DS4") {
    joy_ = std::make_shared<const DS4JoyConverter>(*msg);
  } else if (joy_type_ == "XBOX") {
    joy_ = std::make_shared<const XBOXJoyConverter>(*msg);
  } else {
    joy_ = std::make_shared<const P65JoyConverter>(*msg);
  }

  if (joy_->shift_up() || joy_->shift_down() || joy_->shift_drive() || joy_->shift_reverse()) {
    publishShift();
  }

  if (joy_->turn_signal_left() || joy_->turn_signal_right() || joy_->clear_turn_signal()) {
    publishTurnSignal();
  }

  if (joy_->gate_mode()) {
    publishGateMode();
  }

  if (joy_->autoware_engage() || joy_->autoware_disengage()) {
    publishAutowareEngage();
  }

  if (joy_->vehicle_engage() || joy_->vehicle_disengage()) {
    publishVehicleEngage();
  }

  if (joy_->emergency_stop()) {
    sendEmergencyRequest(true);
  }

  if (joy_->clear_emergency_stop()) {
    sendEmergencyRequest(false);
  }
}

void AutowareJoyControllerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist->header = msg->header;
  twist->twist = msg->twist.twist;

  twist_ = twist;
}

bool AutowareJoyControllerNode::isDataReady()
{
  // Joy
  {
    if (!joy_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for joy msg...");
      return false;
    }

    constexpr auto timeout = 2.0;
    const auto time_diff = this->now() - last_joy_received_time_;
    if (time_diff.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "joy msg is timeout");
      return false;
    }
  }

  // Twist
  if (!raw_control_) {
    if (!twist_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for twist msg...");
      return false;
    }

    constexpr auto timeout = 0.5;
    const auto time_diff = this->now() - twist_->header.stamp;
    if (time_diff.seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
        "twist msg is timeout");
      return false;
    }
  }

  return true;
}

void AutowareJoyControllerNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  publishControlCommand();
  publishExternalControlCommand();
  publishHeartbeat();
}

void AutowareJoyControllerNode::publishControlCommand()
{
  autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
  cmd.stamp = this->now();
  {
    cmd.lateral.steering_tire_angle = steer_ratio_ * joy_->steer();
    cmd.lateral.steering_tire_rotation_rate = steering_angle_velocity_;

    if (joy_->accel()) {
      cmd.longitudinal.acceleration = accel_ratio_ * joy_->accel();
      cmd.longitudinal.speed =
        twist_->twist.linear.x + velocity_gain_ * cmd.longitudinal.acceleration;
      cmd.longitudinal.speed =
        std::min(cmd.longitudinal.speed, static_cast<float>(max_forward_velocity_));
    }

    if (joy_->brake()) {
      cmd.longitudinal.speed = 0.0;
      cmd.longitudinal.acceleration = -brake_ratio_ * joy_->brake();
    }

    // Backward
    if (joy_->accel() && joy_->brake()) {
      cmd.longitudinal.acceleration = backward_accel_ratio_ * joy_->accel();
      cmd.longitudinal.speed =
        twist_->twist.linear.x - velocity_gain_ * cmd.longitudinal.acceleration;
      cmd.longitudinal.speed =
        std::max(cmd.longitudinal.speed, static_cast<float>(-max_backward_velocity_));
    }
  }

  pub_control_command_->publish(cmd);
  prev_control_command_ = cmd;
}

void AutowareJoyControllerNode::publishExternalControlCommand()
{
  tier4_external_api_msgs::msg::ControlCommandStamped cmd_stamped;
  cmd_stamped.stamp = this->now();
  {
    auto & cmd = cmd_stamped.control;

    cmd.steering_angle = steer_ratio_ * joy_->steer();
    cmd.steering_angle_velocity = steering_angle_velocity_;
    cmd.throttle =
      accel_ratio_ * calcMapping(static_cast<double>(joy_->accel()), accel_sensitivity_);
    cmd.brake = brake_ratio_ * calcMapping(static_cast<double>(joy_->brake()), brake_sensitivity_);
  }

  pub_external_control_command_->publish(cmd_stamped);
  prev_external_control_command_ = cmd_stamped.control;
}

void AutowareJoyControllerNode::publishShift()
{
  tier4_external_api_msgs::msg::GearShiftStamped gear_shift;
  gear_shift.stamp = this->now();

  if (joy_->shift_up()) {
    gear_shift.gear_shift.data = getUpperShift(prev_shift_);
  }

  if (joy_->shift_down()) {
    gear_shift.gear_shift.data = getLowerShift(prev_shift_);
  }

  if (joy_->shift_drive()) {
    gear_shift.gear_shift.data = GearShift::DRIVE;
  }

  if (joy_->shift_reverse()) {
    gear_shift.gear_shift.data = GearShift::REVERSE;
  }

  RCLCPP_INFO(get_logger(), "GearShift::%s", getShiftName(gear_shift.gear_shift.data));

  pub_shift_->publish(gear_shift);
  prev_shift_ = gear_shift.gear_shift.data;
}

void AutowareJoyControllerNode::publishTurnSignal()
{
  tier4_external_api_msgs::msg::TurnSignalStamped turn_signal;
  turn_signal.stamp = this->now();

  if (joy_->turn_signal_left() && joy_->turn_signal_right()) {
    turn_signal.turn_signal.data = TurnSignal::HAZARD;
  } else if (joy_->turn_signal_left()) {
    turn_signal.turn_signal.data = TurnSignal::LEFT;
  } else if (joy_->turn_signal_right()) {
    turn_signal.turn_signal.data = TurnSignal::RIGHT;
  }

  if (joy_->clear_turn_signal()) {
    turn_signal.turn_signal.data = TurnSignal::NONE;
  }

  RCLCPP_INFO(get_logger(), "TurnSignal::%s", getTurnSignalName(turn_signal.turn_signal.data));

  pub_turn_signal_->publish(turn_signal);
}

void AutowareJoyControllerNode::publishGateMode()
{
  tier4_control_msgs::msg::GateMode gate_mode;

  if (prev_gate_mode_ == GateMode::AUTO) {
    gate_mode.data = GateMode::EXTERNAL;
  }

  if (prev_gate_mode_ == GateMode::EXTERNAL) {
    gate_mode.data = GateMode::AUTO;
  }

  RCLCPP_INFO(get_logger(), "GateMode::%s", getGateModeName(gate_mode.data));

  pub_gate_mode_->publish(gate_mode);
  prev_gate_mode_ = gate_mode.data;
}

void AutowareJoyControllerNode::publishHeartbeat()
{
  tier4_external_api_msgs::msg::Heartbeat heartbeat;
  heartbeat.stamp = this->now();
  pub_heartbeat_->publish(heartbeat);
}

void AutowareJoyControllerNode::sendEmergencyRequest(bool emergency)
{
  RCLCPP_INFO(get_logger(), "%s emergency stop", emergency ? "Set" : "Clear");

  auto request = std::make_shared<tier4_external_api_msgs::srv::SetEmergency::Request>();
  request->emergency = emergency;

  client_emergency_stop_->async_send_request(
    request, [this, emergency](
               rclcpp::Client<tier4_external_api_msgs::srv::SetEmergency>::SharedFuture result) {
      auto response = result.get();
      if (tier4_api_utils::is_success(response->status)) {
        RCLCPP_INFO(get_logger(), "service succeeded");
      } else {
        RCLCPP_WARN(get_logger(), "service failed: %s", response->status.message.c_str());
      }
    });
}

void AutowareJoyControllerNode::publishAutowareEngage()
{
  auto req = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
  if (joy_->autoware_engage()) {
    req->engage = true;
    RCLCPP_INFO(get_logger(), "Autoware Engage");
  }

  if (joy_->autoware_disengage()) {
    req->engage = false;
    RCLCPP_INFO(get_logger(), "Autoware Disengage");
  }

  if (!client_autoware_engage_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "%s is unavailable", client_autoware_engage_->get_service_name());
    return;
  }

  client_autoware_engage_->async_send_request(
    req, [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture result) {
      RCLCPP_INFO(
        get_logger(), "%s: %d, %s", client_autoware_engage_->get_service_name(),
        result.get()->status.code, result.get()->status.message.c_str());
    });
}

void AutowareJoyControllerNode::publishVehicleEngage()
{
  autoware_auto_vehicle_msgs::msg::Engage engage;

  if (joy_->vehicle_engage()) {
    engage.engage = true;
    RCLCPP_INFO(get_logger(), "Vehicle Engage");
  }

  if (joy_->vehicle_disengage()) {
    engage.engage = false;
    RCLCPP_INFO(get_logger(), "Vehicle Disengage");
  }

  pub_vehicle_engage_->publish(engage);
}

void AutowareJoyControllerNode::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&AutowareJoyControllerNode::onTimer, this));
}

AutowareJoyControllerNode::AutowareJoyControllerNode(const rclcpp::NodeOptions & node_options)
: Node("joy_controller", node_options)
{
  // Parameter
  joy_type_ = declare_parameter<std::string>("joy_type");
  update_rate_ = declare_parameter<double>("update_rate");
  accel_ratio_ = declare_parameter<double>("accel_ratio");
  brake_ratio_ = declare_parameter<double>("brake_ratio");
  steer_ratio_ = declare_parameter<double>("steer_ratio");
  steering_angle_velocity_ = declare_parameter<double>("steering_angle_velocity");
  accel_sensitivity_ = declare_parameter<double>("accel_sensitivity");
  brake_sensitivity_ = declare_parameter<double>("brake_sensitivity");
  raw_control_ = declare_parameter<bool>("control_command.raw_control");
  velocity_gain_ = declare_parameter<double>("control_command.velocity_gain");
  max_forward_velocity_ = declare_parameter<double>("control_command.max_forward_velocity");
  max_backward_velocity_ = declare_parameter<double>("control_command.max_backward_velocity");
  backward_accel_ratio_ = declare_parameter<double>("control_command.backward_accel_ratio");

  RCLCPP_INFO(get_logger(), "Joy type: %s", joy_type_.c_str());

  // Callback Groups
  callback_group_subscribers_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  // Subscriber
  sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "input/joy", 1, std::bind(&AutowareJoyControllerNode::onJoy, this, std::placeholders::_1),
    subscriber_option);
  if (!raw_control_) {
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "input/odometry", 1,
      std::bind(&AutowareJoyControllerNode::onOdometry, this, std::placeholders::_1),
      subscriber_option);
  } else {
    twist_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
  }

  // Publisher
  pub_control_command_ =
    this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "output/control_command", 1);
  pub_external_control_command_ =
    this->create_publisher<tier4_external_api_msgs::msg::ControlCommandStamped>(
      "output/external_control_command", 1);
  pub_shift_ =
    this->create_publisher<tier4_external_api_msgs::msg::GearShiftStamped>("output/shift", 1);
  pub_turn_signal_ = this->create_publisher<tier4_external_api_msgs::msg::TurnSignalStamped>(
    "output/turn_signal", 1);
  pub_gate_mode_ = this->create_publisher<tier4_control_msgs::msg::GateMode>("output/gate_mode", 1);
  pub_heartbeat_ =
    this->create_publisher<tier4_external_api_msgs::msg::Heartbeat>("output/heartbeat", 1);
  pub_vehicle_engage_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("output/vehicle_engage", 1);

  // Service Client
  client_emergency_stop_ = this->create_client<tier4_external_api_msgs::srv::SetEmergency>(
    "service/emergency_stop", rmw_qos_profile_services_default, callback_group_services_);
  while (!client_emergency_stop_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for emergency_stop service connection...");
  }

  client_autoware_engage_ =
    this->create_client<tier4_external_api_msgs::srv::Engage>("service/autoware_engage");

  // Timer
  initTimer(1.0 / update_rate_);
}
}  // namespace joy_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(joy_controller::AutowareJoyControllerNode)
