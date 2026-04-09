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

#ifndef ROBOTEQ_INTERFACE__ROBOTEQ_INTERFACE_HPP_
#define ROBOTEQ_INTERFACE__ROBOTEQ_INTERFACE_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "autoware/vehicle_info_utils/vehicle_info_utils.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "roboteq_msgs/msg/roboteq_command_stamped.hpp"
#include "roboteq_msgs/msg/roboteq_status_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"
#include "tier4_vehicle_msgs/msg/battery_status.hpp"
#include "tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp"

class RoboteqInterface : public rclcpp::Node
{
public:
  explicit RoboteqInterface(const rclcpp::NodeOptions & node_options);

private:
  double wheel_base_;
  double wheel_tread_;
  double wheel_radius_;
  double speed_scale_factor_;
  double loop_rate_;
  double control_cmd_timeout_sec_;
  double roboteq_status_timeout_sec_;
  double vehicle_velocity_limit_;
  bool is_emergency_{false};
  rclcpp::Time prev_control_cmd_stamp_{0, 0, RCL_ROS_TIME};
  bool is_control_command_timeout_;

  roboteq_msgs::msg::RoboteqCommandStamped roboteq_right_cmd_, roboteq_left_cmd_;
  roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr right_status_ptr_, left_status_ptr_;
  autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr gear_cmd_ptr_;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr hazard_light_cmd_ptr_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;

  void onAckermannControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onTurnIndicatorsCmd(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void onHazardLightsCmd(
    const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  void onGearCmd(const autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
  void onEmergencyCmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
  void onRightStatus(const roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr msg);
  void onLeftStatus(const roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr msg);
  void convertTwistToWheelsRPM(
    const double trans_vel, const double angular_vel, double * const left_wheel_rpm,
    double * const right_wheel_rpm);
  void onTimer();
  void publishVehicleControlMode();
  void publishCommand();
  void publishZeroCommand();
  void publishVelocityAndSteering(
    roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr left_msg,
    roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr right_msg);
  void publishTurnIndicator(
    roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr left_msg,
    roboteq_msgs::msg::RoboteqStatusStamped::ConstSharedPtr right_msg);

  // Diagnostics
  void setupDiagnosticUpdater();
  void checkControlCommand(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkLeftRoboteqConnection(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkRightRoboteqConnection(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkLeftRoboteqErrors(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkRightRoboteqErrors(diagnostic_updater::DiagnosticStatusWrapper & stat);
  diagnostic_updater::Updater diagnostic_updater_{this};

  // Subscribe from Autoware
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_lights_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_sub_;

  // Subscribe from roboteq
  rclcpp::Subscription<roboteq_msgs::msg::RoboteqStatusStamped>::SharedPtr left_status_sub_;
  rclcpp::Subscription<roboteq_msgs::msg::RoboteqStatusStamped>::SharedPtr right_status_sub_;

  // Publish to roboteq
  rclcpp::Publisher<roboteq_msgs::msg::RoboteqCommandStamped>::SharedPtr roboteq_cmd_right_pub_;
  rclcpp::Publisher<roboteq_msgs::msg::RoboteqCommandStamped>::SharedPtr roboteq_cmd_left_pub_;
  // Publish to Autoware
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr
    velocity_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    steering_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    turn_indicators_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    hazard_lights_status_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_status_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr velocity_kmph_status_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr
    steering_wheel_deg_status_pub_;
  rclcpp::Publisher<tier4_vehicle_msgs::msg::BatteryStatus>::SharedPtr battery_charge_status_pub_;
};

#endif  // ROBOTEQ_INTERFACE__ROBOTEQ_INTERFACE_HPP_
