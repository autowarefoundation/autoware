// Copyright 2021 Tier IV, Inc.
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

#include "roboteq_interface/roboteq_interface.hpp"

RoboteqInterface::RoboteqInterface(const rclcpp::NodeOptions & node_options)
: Node("roboteq_interface", node_options)
{
  using std::placeholders::_1;

  // Parameters
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  wheel_base_ = vehicle_info.wheel_base_m;
  wheel_tread_ = vehicle_info.wheel_tread_m;
  wheel_radius_ = vehicle_info.wheel_radius_m;
  speed_scale_factor_ = declare_parameter<double>("speed_scale_factor", 1.0);
  loop_rate_ = declare_parameter<double>("loop_rate", 50.0);
  control_cmd_timeout_sec_ = declare_parameter<double>("control_cmd_timeout_sec", 1.0);
  roboteq_status_timeout_sec_ = declare_parameter<double>("roboteq_status_timeout_sec", 0.5);
  vehicle_velocity_limit_ = declare_parameter<double>("vehicle_velocity_limit", 0.83);

  // Subscribe from Autoware
  control_cmd_sub_ =
    this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", 1,
      std::bind(&RoboteqInterface::onAckermannControlCmd, this, _1));

  gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "/control/command/gear_cmd", 1, std::bind(&RoboteqInterface::onGearCmd, this, _1));

  turn_indicators_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      "/control/command/turn_indicators_cmd", 1,
      std::bind(&RoboteqInterface::onTurnIndicatorsCmd, this, _1));

  hazard_lights_cmd_sub_ =
    this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
      "/control/command/hazard_lights_cmd", 1,
      std::bind(&RoboteqInterface::onHazardLightsCmd, this, _1));

  emergency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
    "/control/command/emergency_cmd", 1, std::bind(&RoboteqInterface::onEmergencyCmd, this, _1));

  // Subscribe from roboteq
  left_status_sub_ = this->create_subscription<roboteq_msgs::msg::RoboteqStatusStamped>(
    "/roboteq/left/status", 1, std::bind(&RoboteqInterface::onLeftStatus, this, _1));

  right_status_sub_ = this->create_subscription<roboteq_msgs::msg::RoboteqStatusStamped>(
    "/roboteq/right/status", 1, std::bind(&RoboteqInterface::onRightStatus, this, _1));

  // Publish to roboteq
  roboteq_cmd_left_pub_ =
    this->create_publisher<roboteq_msgs::msg::RoboteqCommandStamped>("/roboteq/left/command", 1);
  roboteq_cmd_right_pub_ =
    this->create_publisher<roboteq_msgs::msg::RoboteqCommandStamped>("/roboteq/right/command", 1);
  // Publish to autoware
  velocity_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "/vehicle/status/velocity_status", 1);
  steering_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
    "/vehicle/status/steering_status", 1);
  gear_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::GearReport>(
    "/vehicle/status/gear_status", 1);
  turn_indicators_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>(
      "/vehicle/status/turn_indicators_status", 1);
  hazard_lights_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>(
      "/vehicle/status/hazard_lights_status", 1);
  control_mode_status_pub_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      "/vehicle/status/control_mode", 1);
  velocity_kmph_status_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/velocity_kmph", 1);
  steering_wheel_deg_status_pub_ = this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>(
    "/vehicle/status/steering_wheel_deg", 1);
  battery_charge_status_pub_ = this->create_publisher<tier4_vehicle_msgs::msg::BatteryStatus>(
    "/vehicle/status/battery_charge", 1);

  setupDiagnosticUpdater();

  // Timer
  const auto period_ns = rclcpp::Rate(loop_rate_).period();
  cmd_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&RoboteqInterface::onTimer, this));
}

void RoboteqInterface::onTimer()
{
  diagnostic_updater_.force_update();

  publishVehicleControlMode();
  if (is_control_command_timeout_) {
    publishZeroCommand();
  } else {
    publishCommand();
  }
}

void RoboteqInterface::publishZeroCommand()
{
  using roboteq_msgs::msg::DigitalOutCommand;
  roboteq_right_cmd_.stamp = this->now();
  roboteq_left_cmd_.stamp = this->now();
  roboteq_left_cmd_.command.speed = 0.0;
  roboteq_right_cmd_.command.speed = 0.0;
  roboteq_right_cmd_.command.digital_output.data.at(0) = DigitalOutCommand::LOW;
  roboteq_left_cmd_.command.digital_output.data.at(0) = DigitalOutCommand::LOW;
  roboteq_cmd_right_pub_->publish(roboteq_right_cmd_);
  roboteq_cmd_left_pub_->publish(roboteq_left_cmd_);
}

void RoboteqInterface::publishCommand()
{
  using roboteq_msgs::msg::DigitalOutCommand;
  if (roboteq_right_cmd_.command.speed == 0 && roboteq_left_cmd_.command.speed == 0) {
    roboteq_right_cmd_.command.digital_output.data.at(0) = DigitalOutCommand::LOW;
    roboteq_left_cmd_.command.digital_output.data.at(0) = DigitalOutCommand::LOW;
  } else {
    roboteq_right_cmd_.command.digital_output.data.at(0) = DigitalOutCommand::HIGH;
    roboteq_left_cmd_.command.digital_output.data.at(0) = DigitalOutCommand::HIGH;
  }

  roboteq_cmd_right_pub_->publish(roboteq_right_cmd_);
  roboteq_cmd_left_pub_->publish(roboteq_left_cmd_);
}

void RoboteqInterface::publishVehicleControlMode()
{
  auto msg = autoware_auto_vehicle_msgs::msg::ControlModeReport{};
  msg.stamp = this->now();
  msg.mode = autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  control_mode_status_pub_->publish(msg);
}

void RoboteqInterface::publishVelocityAndSteering(
  roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr left_msg,
  roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr right_msg)
{
  const double rpm_unit = 2 * wheel_radius_ * M_PI / 60.0;

  // calcVehicleTwist
  autoware_auto_vehicle_msgs::msg::VelocityReport twist;
  twist.header.stamp = left_msg->stamp;
  twist.header.frame_id = "base_link";
  double left_wheel_vel =
    static_cast<double>(left_msg->status.motor_speed_rpm) * rpm_unit * speed_scale_factor_;
  double right_wheel_vel =
    static_cast<double>(right_msg->status.motor_speed_rpm) * rpm_unit * speed_scale_factor_;
  twist.longitudinal_velocity = (left_wheel_vel + right_wheel_vel) / 2;
  twist.heading_rate = (right_wheel_vel - left_wheel_vel) / wheel_tread_;
  velocity_status_pub_->publish(twist);

  tier4_debug_msgs::msg::Float32Stamped velocity_kmph_msg;
  velocity_kmph_msg.data = twist.longitudinal_velocity * 3.6;
  velocity_kmph_msg.stamp = this->now();
  velocity_kmph_status_pub_->publish(velocity_kmph_msg);

  autoware_auto_vehicle_msgs::msg::SteeringReport steer_msg;
  steer_msg.stamp = left_msg->stamp;
  steer_msg.steering_tire_angle =
    twist.longitudinal_velocity != 0.0
      ? std::atan(twist.heading_rate * wheel_base_ / twist.longitudinal_velocity)
      : 0.0;
  steering_status_pub_->publish(steer_msg);

  tier4_debug_msgs::msg::Float32Stamped steer_wheel_deg_msg;
  steer_wheel_deg_msg.data = steer_msg.steering_tire_angle * 180.0 / M_PI;
  steer_wheel_deg_msg.stamp = this->now();
  steering_wheel_deg_status_pub_->publish(steer_wheel_deg_msg);
}

void RoboteqInterface::publishTurnIndicator(
  roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr left_msg,
  roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr right_msg)
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsReport;
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;

  auto hazard_report_msg = HazardLightsReport{};
  hazard_report_msg.stamp = this->now();
  auto turn_indicator_report_msg = TurnIndicatorsReport{};
  turn_indicator_report_msg.stamp = this->now();

  const auto & left_digital_output = left_msg->status.digital_output;
  const auto & right_digital_output = right_msg->status.digital_output;

  if (left_digital_output.at(1) == 0 && right_digital_output.at(1) == 0) {
    turn_indicator_report_msg.report = TurnIndicatorsReport::DISABLE;
    hazard_report_msg.report = HazardLightsReport::DISABLE;
  } else if (left_digital_output.at(1) == 1 && right_digital_output.at(1) == 0) {
    turn_indicator_report_msg.report = TurnIndicatorsReport::ENABLE_LEFT;
    hazard_report_msg.report = HazardLightsReport::DISABLE;
  } else if (left_digital_output.at(1) == 0 && right_digital_output.at(1) == 1) {
    turn_indicator_report_msg.report = TurnIndicatorsReport::ENABLE_RIGHT;
    hazard_report_msg.report = HazardLightsReport::DISABLE;
  } else if (left_digital_output.at(1) == 1 && right_digital_output.at(1) == 1) {
    turn_indicator_report_msg.report = TurnIndicatorsReport::DISABLE;
    hazard_report_msg.report = HazardLightsReport::ENABLE;
  }

  turn_indicators_status_pub_->publish(turn_indicator_report_msg);
  hazard_lights_status_pub_->publish(hazard_report_msg);
}

void RoboteqInterface::onAckermannControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;

  if (!gear_cmd_ptr_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "gear command is not subscribed");
    return;
  }

  roboteq_right_cmd_.stamp = msg->stamp;
  roboteq_left_cmd_.stamp = msg->stamp;

  double trans_vel = msg->longitudinal.speed;
  if (trans_vel > vehicle_velocity_limit_) {
    trans_vel = vehicle_velocity_limit_;
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *get_clock(), 1000,
      "error: input command over the limit velocity(%f[m/s]), limit to %f[m/s]",
      msg->longitudinal.speed, vehicle_velocity_limit_);
  }
  double angular_vel = trans_vel * std::tan(msg->lateral.steering_tire_angle) / wheel_base_;

  double left_wheel_rpm = 0.0;
  double right_wheel_rpm = 0.0;
  if (!is_emergency_ && gear_cmd_ptr_->command != GearCommand::PARK) {
    convertTwistToWheelsRPM(trans_vel, angular_vel, &left_wheel_rpm, &right_wheel_rpm);
  }

  roboteq_left_cmd_.command.speed = round(left_wheel_rpm);
  roboteq_right_cmd_.command.speed = round(right_wheel_rpm);

  prev_control_cmd_stamp_ = this->now();
}

void RoboteqInterface::onTurnIndicatorsCmd(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg)
{
  using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
  using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
  using roboteq_msgs::msg::DigitalOutCommand;

  if (!hazard_light_cmd_ptr_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "hazard light command is not subscribed");
    return;
  }

  if (hazard_light_cmd_ptr_->command == HazardLightsCommand::ENABLE) {
    roboteq_right_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::HIGH;
    roboteq_left_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::HIGH;
  } else {
    switch (msg->command) {
      case TurnIndicatorsCommand::NO_COMMAND:
        roboteq_right_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::LOW;
        roboteq_left_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::LOW;
        break;
      case TurnIndicatorsCommand::DISABLE:
        roboteq_right_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::LOW;
        roboteq_left_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::LOW;
        break;
      case TurnIndicatorsCommand::ENABLE_LEFT:
        roboteq_right_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::LOW;
        roboteq_left_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::HIGH;
        break;
      case TurnIndicatorsCommand::ENABLE_RIGHT:
        roboteq_right_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::HIGH;
        roboteq_left_cmd_.command.digital_output.data.at(1) = DigitalOutCommand::LOW;
        break;
      default:
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *get_clock(), 3000, "error: turn_indicator_cmd = %d", msg->command);
        break;
    }
  }
}

void RoboteqInterface::onHazardLightsCmd(
  const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_light_cmd_ptr_ = msg;
}

void RoboteqInterface::onGearCmd(
  const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
{
  using autoware_auto_vehicle_msgs::msg::GearCommand;
  using autoware_auto_vehicle_msgs::msg::GearReport;

  gear_cmd_ptr_ = msg;

  auto report_msg = GearReport{};
  report_msg.stamp = msg->stamp;
  if (msg->command == GearCommand::PARK) {
    report_msg.report = GearReport::PARK;
  }
  if (msg->command == GearCommand::REVERSE) {
    report_msg.report = GearReport::REVERSE;
  }
  if (msg->command == GearCommand::DRIVE) {
    report_msg.report = GearReport::DRIVE;
  }
  if (msg->command == GearCommand::LOW) {
    report_msg.report = GearReport::LOW;
  }
  gear_status_pub_->publish(report_msg);
}

void RoboteqInterface::onEmergencyCmd(
  const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg)
{
  is_emergency_ = msg->emergency;
}

void RoboteqInterface::convertTwistToWheelsRPM(
  const double trans_vel, const double angular_vel, double * const left_wheel_rpm,
  double * const right_wheel_rpm)
{
  const double rpm_unit = 2 * wheel_radius_ * M_PI / 60.0;

  double left_wheel_vel = (fabs(angular_vel) > 0)
                            ? ((trans_vel / angular_vel) - wheel_tread_ / 2.0) * angular_vel
                            : trans_vel;
  double right_wheel_vel = (fabs(angular_vel) > 0)
                             ? ((trans_vel / angular_vel) + wheel_tread_ / 2.0) * angular_vel
                             : trans_vel;

  *left_wheel_rpm = left_wheel_vel / rpm_unit;
  *right_wheel_rpm = right_wheel_vel / rpm_unit;
}

void RoboteqInterface::onLeftStatus(
  const roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr msg)
{
  left_status_ptr_ = msg;

  if (!right_status_ptr_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "right motor status is not subscribed");
    return;
  }

  publishVelocityAndSteering(msg, right_status_ptr_);
  publishTurnIndicator(msg, right_status_ptr_);
}

void RoboteqInterface::onRightStatus(
  const roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr msg)
{
  right_status_ptr_ = msg;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RoboteqInterface)
