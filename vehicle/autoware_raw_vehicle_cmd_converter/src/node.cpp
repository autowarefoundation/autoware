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

#include "autoware_raw_vehicle_cmd_converter/node.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::raw_vehicle_cmd_converter
{
RawVehicleCommandConverterNode::RawVehicleCommandConverterNode(
  const rclcpp::NodeOptions & node_options)
: Node("autoware_raw_vehicle_cmd_converter_node", node_options)
{
  using std::placeholders::_1;
  /* parameters for accel/brake map */
  const auto csv_path_accel_map = declare_parameter<std::string>("csv_path_accel_map");
  const auto csv_path_brake_map = declare_parameter<std::string>("csv_path_brake_map");
  convert_accel_cmd_ = declare_parameter<bool>("convert_accel_cmd");
  convert_brake_cmd_ = declare_parameter<bool>("convert_brake_cmd");
  max_accel_cmd_ = declare_parameter<double>("max_throttle");
  max_brake_cmd_ = declare_parameter<double>("max_brake");
  max_steer_cmd_ = declare_parameter<double>("max_steer");
  min_steer_cmd_ = declare_parameter<double>("min_steer");
  is_debugging_ = declare_parameter<bool>("is_debugging");
  // for steering steer controller
  use_steer_ff_ = declare_parameter<bool>("use_steer_ff");
  use_steer_fb_ = declare_parameter<bool>("use_steer_fb");
  if (convert_accel_cmd_) {
    if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map, true)) {
      throw std::invalid_argument("Accel map is invalid.");
    }
  }
  if (convert_brake_cmd_) {
    if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map, true)) {
      throw std::invalid_argument("Brake map is invalid.");
    }
  }
  if (declare_parameter<bool>("convert_steer_cmd")) {
    convert_steer_cmd_method_ = declare_parameter<std::string>("convert_steer_cmd_method", "vgr");
    if (convert_steer_cmd_method_.value() == "vgr") {
      const double a = declare_parameter<double>("vgr_coef_a");
      const double b = declare_parameter<double>("vgr_coef_b");
      const double c = declare_parameter<double>("vgr_coef_c");
      vgr_.setCoefficients(a, b, c);
    } else if (convert_steer_cmd_method_.value() == "steer_map") {
      const auto csv_path_steer_map = declare_parameter<std::string>("csv_path_steer_map");
      if (!steer_map_.readSteerMapFromCSV(csv_path_steer_map, true)) {
        throw std::invalid_argument("Steer map is invalid.");
      }
      const auto kp_steer{declare_parameter<double>("steer_pid.kp")};
      const auto ki_steer{declare_parameter<double>("steer_pid.ki")};
      const auto kd_steer{declare_parameter<double>("steer_pid.kd")};
      const auto max_ret_steer{declare_parameter<double>("steer_pid.max")};
      const auto min_ret_steer{declare_parameter<double>("steer_pid.min")};
      const auto max_ret_p_steer{declare_parameter<double>("steer_pid.max_p")};
      const auto min_ret_p_steer{declare_parameter<double>("steer_pid.min_p")};
      const auto max_ret_i_steer{declare_parameter<double>("steer_pid.max_i")};
      const auto min_ret_i_steer{declare_parameter<double>("steer_pid.min_i")};
      const auto max_ret_d_steer{declare_parameter<double>("steer_pid.max_d")};
      const auto min_ret_d_steer{declare_parameter<double>("steer_pid.min_d")};
      const auto invalid_integration_decay{
        declare_parameter<double>("steer_pid.invalid_integration_decay")};
      steer_pid_.setDecay(invalid_integration_decay);
      steer_pid_.setGains(kp_steer, ki_steer, kd_steer);
      steer_pid_.setLimits(
        max_ret_steer, min_ret_steer, max_ret_p_steer, min_ret_p_steer, max_ret_i_steer,
        min_ret_i_steer, max_ret_d_steer, min_ret_d_steer);
      steer_pid_.setInitialized();
    } else {
      throw std::invalid_argument("Invalid steer conversion method.");
    }
  }

  // NOTE: The steering status can be published from the vehicle side or converted in this node.
  convert_actuation_to_steering_status_ =
    declare_parameter<bool>("convert_actuation_to_steering_status", false);
  if (convert_actuation_to_steering_status_) {
    pub_steering_status_ = create_publisher<Steering>("~/output/steering_status", 1);
  } else {
    // NOTE: Polling subscriber requires specifying the topic name at declaration,
    // so use a normal callback subscriber.
    sub_steering_ = create_subscription<Steering>(
      "~/input/steering", 1, std::bind(&RawVehicleCommandConverterNode::onSteering, this, _1));
  }

  // NOTE: some vehicles do not publish actuation status. To handle this, subscribe only when the
  // option is specified.
  const bool use_vgr =
    convert_steer_cmd_method_.has_value() && convert_steer_cmd_method_.value() == "vgr";
  need_to_subscribe_actuation_status_ = convert_actuation_to_steering_status_ || use_vgr;
  if (need_to_subscribe_actuation_status_) {
    sub_actuation_status_ = create_subscription<ActuationStatusStamped>(
      "~/input/actuation_status", 1,
      std::bind(&RawVehicleCommandConverterNode::onActuationStatus, this, _1));
  }

  sub_control_cmd_ = create_subscription<Control>(
    "~/input/control_cmd", 1, std::bind(&RawVehicleCommandConverterNode::onControlCmd, this, _1));

  pub_actuation_cmd_ = create_publisher<ActuationCommandStamped>("~/output/actuation_cmd", 1);
  debug_pub_steer_pid_ = create_publisher<Float32MultiArrayStamped>(
    "/vehicle/raw_vehicle_cmd_converter/debug/steer_pid", 1);

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
}

void RawVehicleCommandConverterNode::publishActuationCmd()
{
  if (!current_twist_ptr_ || !control_cmd_ptr_ || !current_steer_ptr_) {
    RCLCPP_WARN_EXPRESSION(
      get_logger(), is_debugging_, "some pointers are null: %s, %s, %s",
      !current_twist_ptr_ ? "twist" : "", !control_cmd_ptr_ ? "cmd" : "",
      !current_steer_ptr_ ? "steer" : "");
    return;
  }

  if (need_to_subscribe_actuation_status_) {
    if (!actuation_status_ptr_) {
      RCLCPP_WARN_EXPRESSION(get_logger(), is_debugging_, "actuation status is null");
      return;
    }
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
  } else if (acc < 0) {
    // if conversion is disabled use negative acceleration as brake cmd
    desired_brake_cmd = -acc;
  }
  if (!convert_steer_cmd_method_.has_value()) {
    // if conversion is disabled use steering angle as steer cmd
    desired_steer_cmd = steer;
  } else if (convert_steer_cmd_method_.value() == "vgr") {
    // NOTE: When using variable gear ratio,
    // the actuation cmd is the steering wheel angle,
    // and the actuation_status is also the steering wheel angle.
    const double current_steer_wheel = actuation_status_ptr_->status.steer_status;
    const double adaptive_gear_ratio = vgr_.calculateVariableGearRatio(vel, current_steer_wheel);
    desired_steer_cmd = steer * adaptive_gear_ratio;
  } else if (convert_steer_cmd_method_.value() == "steer_map") {
    desired_steer_cmd = calculateSteerFromMap(vel, steer, steer_rate);
  }
  actuation_cmd.header.frame_id = "base_link";
  actuation_cmd.header.stamp = control_cmd_ptr_->stamp;
  actuation_cmd.actuation.accel_cmd = desired_accel_cmd;
  actuation_cmd.actuation.brake_cmd = desired_brake_cmd;
  actuation_cmd.actuation.steer_cmd = desired_steer_cmd;
  pub_actuation_cmd_->publish(actuation_cmd);
}

double RawVehicleCommandConverterNode::calculateSteerFromMap(
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
    dt = 0.1;  // set ordinary delta time instead
  }
  prev_time_steer_calculation_ = current_time;
  // feed-forward
  if (use_steer_ff_) {
    steer_map_.getSteer(steer_rate, *current_steer_ptr_, ff_value);
  }
  // feed-back
  if (use_steer_fb_) {
    fb_value =
      steer_pid_.calculateFB(steering, dt, vel, *current_steer_ptr_, pid_contributions, pid_errors);
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

void RawVehicleCommandConverterNode::onControlCmd(const Control::ConstSharedPtr msg)
{
  const auto odometry_msg = sub_odometry_.takeData();
  if (odometry_msg) {
    current_twist_ptr_ = std::make_unique<TwistStamped>();
    current_twist_ptr_->header = odometry_msg->header;
    current_twist_ptr_->twist = odometry_msg->twist.twist;
  }
  control_cmd_ptr_ = msg;
  publishActuationCmd();
}

void RawVehicleCommandConverterNode::onSteering(const Steering::ConstSharedPtr msg)
{
  current_steer_ptr_ = std::make_unique<double>(msg->steering_tire_angle);
}

void RawVehicleCommandConverterNode::onActuationStatus(
  const ActuationStatusStamped::ConstSharedPtr msg)
{
  actuation_status_ptr_ = msg;

  if (!convert_actuation_to_steering_status_ || !convert_steer_cmd_method_.has_value()) {
    return;
  }

  // calculate steering status from actuation status
  const auto odometry_msg = sub_odometry_.takeData();
  if (odometry_msg) {
    if (convert_steer_cmd_method_.value() == "vgr") {
      current_twist_ptr_ = std::make_unique<TwistStamped>();
      current_twist_ptr_->header = odometry_msg->header;
      current_twist_ptr_->twist = odometry_msg->twist.twist;
      current_steer_ptr_ = std::make_unique<double>(vgr_.calculateSteeringTireState(
        current_twist_ptr_->twist.linear.x, actuation_status_ptr_->status.steer_status));
      Steering steering_msg{};
      steering_msg.steering_tire_angle = *current_steer_ptr_;
      pub_steering_status_->publish(steering_msg);
    } else if (convert_steer_cmd_method_.value() == "steer_map") {
      throw std::domain_error(
        "Steer map conversion is not supported for actuation status. Please "
        "use vgr conversion method or set convert_actuation_to_steering to "
        "false.");
    }
  }
}
}  // namespace autoware::raw_vehicle_cmd_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::raw_vehicle_cmd_converter::RawVehicleCommandConverterNode)
