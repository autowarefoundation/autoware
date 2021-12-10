//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "raw_vehicle_cmd_converter/node.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
RawVehicleCommandConverterNode::RawVehicleCommandConverterNode(
  const rclcpp::NodeOptions & node_options)
: Node("raw_vehicle_cmd_converter_node", node_options)
{
  using std::placeholders::_1;
  /* parameters for accel/brake map */
  const auto csv_path_accel_map = declare_parameter("csv_path_accel_map", std::string("empty"));
  const auto csv_path_brake_map = declare_parameter("csv_path_brake_map", std::string("empty"));
  const auto csv_path_steer_map = declare_parameter("csv_path_steer_map", std::string("empty"));
  convert_accel_cmd_ = declare_parameter("convert_accel_cmd", true);
  convert_brake_cmd_ = declare_parameter("convert_brake_cmd", true);
  convert_steer_cmd_ = declare_parameter("convert_steer_cmd", true);
  max_accel_cmd_ = declare_parameter("max_throttle", 0.2);
  max_brake_cmd_ = declare_parameter("max_brake", 0.8);
  max_steer_cmd_ = declare_parameter("max_steer", 10.0);
  min_steer_cmd_ = declare_parameter("min_steer", -10.0);
  is_debugging_ = declare_parameter("is_debugging", false);
  // for steering steer controller
  use_steer_ff_ = declare_parameter("use_steer_ff", true);
  use_steer_fb_ = declare_parameter("use_steer_fb", true);
  const auto kp_steer{declare_parameter("steer_pid.kp", 150.0)};
  const auto ki_steer{declare_parameter("steer_pid.ki", 15.0)};
  const auto kd_steer{declare_parameter("steer_pid.kd", 0.0)};
  const auto max_ret_steer{declare_parameter("steer_pid.max", 8.0)};
  const auto min_ret_steer{declare_parameter("steer_pid.min", -8.0)};
  const auto max_ret_p_steer{declare_parameter("steer_pid.max_p", 8.0)};
  const auto min_ret_p_steer{declare_parameter("steer_pid.min_p", -8.0)};
  const auto max_ret_i_steer{declare_parameter("steer_pid.max_i", 8.0)};
  const auto min_ret_i_steer{declare_parameter("steer_pid.min_i", -8.0)};
  const auto max_ret_d_steer{declare_parameter("steer_pid.max_d", 0.0)};
  const auto min_ret_d_steer{declare_parameter("steer_pid.min_d", 0.0)};
  const auto invalid_integration_decay{
    declare_parameter("steer_pid.invalid_integration_decay", 0.97)};
  ff_map_initialized_ = true;
  if (convert_accel_cmd_) {
    if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map)) {
      RCLCPP_ERROR(
        get_logger(), "Cannot read accelmap. csv path = %s. stop calculation.",
        csv_path_accel_map.c_str());
      ff_map_initialized_ = false;
    }
  }
  if (convert_brake_cmd_) {
    if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map)) {
      RCLCPP_ERROR(
        get_logger(), "Cannot read brakemap. csv path = %s. stop calculation.",
        csv_path_brake_map.c_str());
      ff_map_initialized_ = false;
    }
  }
  if (convert_steer_cmd_) {
    steer_controller_.setDecay(invalid_integration_decay);
    if (!steer_controller_.setFFMap(csv_path_steer_map)) {
      RCLCPP_ERROR(
        get_logger(), "Cannot read steer map. csv path = %s. stop calculation.",
        csv_path_steer_map.c_str());
      ff_map_initialized_ = false;
    }
    steer_controller_.setFBGains(kp_steer, ki_steer, kd_steer);
    steer_controller_.setFBLimits(
      max_ret_steer, min_ret_steer, max_ret_p_steer, min_ret_p_steer, max_ret_i_steer,
      min_ret_i_steer, max_ret_d_steer, min_ret_d_steer);
  }
  pub_actuation_cmd_ = create_publisher<ActuationCommandStamped>("~/output/actuation_cmd", 1);
  sub_control_cmd_ = create_subscription<AckermannControlCommand>(
    "~/input/control_cmd", 1, std::bind(&RawVehicleCommandConverterNode::onControlCmd, this, _1));
  sub_velocity_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&RawVehicleCommandConverterNode::onVelocity, this, _1));
  sub_steering_ = create_subscription<Steering>(
    "~/input/steering", 1, std::bind(&RawVehicleCommandConverterNode::onSteering, this, _1));
  debug_pub_steer_pid_ = create_publisher<Float32MultiArrayStamped>(
    "/vehicle/raw_vehicle_cmd_converter/debug/steer_pid", 1);
}

void RawVehicleCommandConverterNode::publishActuationCmd()
{
  if (!ff_map_initialized_) {
    RCLCPP_WARN_EXPRESSION(get_logger(), is_debugging_, "ff map is not initialized");
    return;
  }
  if (!current_twist_ptr_ || !control_cmd_ptr_ || !current_steer_ptr_) {
    RCLCPP_WARN_EXPRESSION(
      get_logger(), is_debugging_, "some of twist/control_cmd/steer pointer is null");
    return;
  }
  double desired_accel_cmd = 0.0;
  double desired_brake_cmd = 0.0;
  double desired_steer_cmd = 0.0;
  ActuationCommandStamped actuation_cmd;
  const double acc = control_cmd_ptr_->longitudinal.acceleration;
  const double vel = current_twist_ptr_->twist.linear.x;
  const double steer = control_cmd_ptr_->lateral.steering_tire_angle;
  const double steer_rate = control_cmd_ptr_->lateral.steering_tire_rotation_rate;
  bool accel_cmd_is_zero = true;
  if (convert_accel_cmd_) {
    desired_accel_cmd = calculateAccelMap(vel, acc, accel_cmd_is_zero);
  } else {
    // if conversion is disabled use acceleration as actuation cmd
    desired_accel_cmd = (acc >= 0) ? acc : 0;
  }
  if (convert_brake_cmd_) {
    if (accel_cmd_is_zero) {
      desired_brake_cmd = calculateBrakeMap(vel, acc);
    }
  } else {
    // if conversion is disabled use acceleration as brake cmd
    desired_brake_cmd = (acc < 0) ? acc : 0;
  }
  if (convert_steer_cmd_) {
    desired_steer_cmd = calculateSteer(vel, steer, steer_rate);
  } else {
    // if conversion is disabled use steering angle as steer cmd
    desired_steer_cmd = steer;
  }
  actuation_cmd.header.frame_id = "base_link";
  actuation_cmd.header.stamp = control_cmd_ptr_->stamp;
  actuation_cmd.actuation.accel_cmd = desired_accel_cmd;
  actuation_cmd.actuation.brake_cmd = desired_brake_cmd;
  actuation_cmd.actuation.steer_cmd = desired_steer_cmd;
  pub_actuation_cmd_->publish(actuation_cmd);
}

double RawVehicleCommandConverterNode::calculateSteer(
  const double vel, const double steering, const double steer_rate)
{
  double steering_output = 0;
  double ff_value = 0;
  double fb_value = 0;
  std::vector<double> pid_contributions(3, 0.0);
  std::vector<double> pid_errors(3, 0.0);
  rclcpp::Time current_time = this->now();
  double dt = (current_time - prev_time_steer_calculation_).seconds();
  if (std::abs(dt) > 1.0) {
    RCLCPP_WARN_EXPRESSION(get_logger(), is_debugging_, "ignore old topic");
    dt = 0.0;
  }
  prev_time_steer_calculation_ = current_time;
  // feed-forward
  if (use_steer_ff_) {
    ff_value = steer_controller_.calcFFSteer(steer_rate, *current_steer_ptr_);
  }
  // feedback
  if (use_steer_fb_) {
    fb_value = steer_controller_.calcFBSteer(
      steering, dt, vel, *current_steer_ptr_, pid_contributions, pid_errors);
  }
  steering_output = ff_value + fb_value;
  // for steer debugging
  {
    debug_steer_.setValues(DebugValues::TYPE::CURR_TIME, current_time.seconds());
    debug_steer_.setValues(DebugValues::TYPE::P, pid_contributions.at(0));
    debug_steer_.setValues(DebugValues::TYPE::I, pid_contributions.at(1));
    debug_steer_.setValues(DebugValues::TYPE::D, pid_contributions.at(2));
    debug_steer_.setValues(DebugValues::TYPE::FF, ff_value);
    debug_steer_.setValues(DebugValues::TYPE::FB, fb_value);
    debug_steer_.setValues(DebugValues::TYPE::STEER, steering_output);
    debug_steer_.setValues(DebugValues::TYPE::ERROR_P, pid_errors.at(0));
    debug_steer_.setValues(DebugValues::TYPE::ERROR_I, pid_errors.at(1));
    debug_steer_.setValues(DebugValues::TYPE::ERROR_D, pid_errors.at(2));
    tier4_debug_msgs::msg::Float32MultiArrayStamped msg{};
    for (const auto & v : debug_steer_.getValues()) {
      msg.data.push_back(v);
    }
    msg.stamp = this->now();
    debug_pub_steer_pid_->publish(msg);
  }
  steering_output = std::max(std::min(max_steer_cmd_, steering_output), min_steer_cmd_);
  return steering_output;
}

double RawVehicleCommandConverterNode::calculateAccelMap(
  const double current_velocity, const double desired_acc, bool & accel_cmd_is_zero)
{
  double desired_accel_cmd = 0;
  if (!accel_map_.getThrottle(desired_acc, std::abs(current_velocity), desired_accel_cmd)) {
    desired_accel_cmd = 0;
  } else {
    accel_cmd_is_zero = false;
  }
  desired_accel_cmd = std::min(std::max(desired_accel_cmd, 0.0), max_accel_cmd_);
  return desired_accel_cmd;
}

double RawVehicleCommandConverterNode::calculateBrakeMap(
  const double current_velocity, const double desired_acc)
{
  double desired_brake_cmd = 0;
  brake_map_.getBrake(desired_acc, std::abs(current_velocity), desired_brake_cmd);
  desired_brake_cmd = std::min(std::max(desired_brake_cmd, 0.0), max_brake_cmd_);
  return desired_brake_cmd;
}

void RawVehicleCommandConverterNode::onSteering(const Steering::ConstSharedPtr msg)
{
  current_steer_ptr_ = std::make_unique<double>(msg->steering_tire_angle);
}

void RawVehicleCommandConverterNode::onVelocity(const Odometry::ConstSharedPtr msg)
{
  current_twist_ptr_ = std::make_unique<TwistStamped>();
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;
}

void RawVehicleCommandConverterNode::onControlCmd(const AckermannControlCommand::ConstSharedPtr msg)
{
  control_cmd_ptr_ = msg;
  publishActuationCmd();
}
}  // namespace raw_vehicle_cmd_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(raw_vehicle_cmd_converter::RawVehicleCommandConverterNode)
