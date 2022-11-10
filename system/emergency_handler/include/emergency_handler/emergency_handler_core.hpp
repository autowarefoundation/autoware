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

#ifndef EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_
#define EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_

// Core
#include <memory>
#include <string>

// Autoware
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_system_msgs/msg/hazard_status_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS2 core
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

struct HazardLampPolicy
{
  bool emergency;
};

struct Param
{
  int update_rate;
  double timeout_hazard_status;
  double timeout_takeover_request;
  bool use_takeover_request;
  bool use_parking_after_stopped;
  bool use_comfortable_stop;
  HazardLampPolicy turning_hazard_on{};
};

class EmergencyHandler : public rclcpp::Node
{
public:
  EmergencyHandler();

private:
  // Subscribers
  rclcpp::Subscription<autoware_auto_system_msgs::msg::HazardStatusStamped>::SharedPtr
    sub_hazard_status_stamped_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_prev_control_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    sub_control_mode_;
  rclcpp::Subscription<tier4_system_msgs::msg::MrmBehaviorStatus>::SharedPtr
    sub_mrm_comfortable_stop_status_;
  rclcpp::Subscription<tier4_system_msgs::msg::MrmBehaviorStatus>::SharedPtr
    sub_mrm_emergency_stop_status_;

  autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr hazard_status_stamped_;
  autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr prev_control_command_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr control_mode_;
  tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr mrm_comfortable_stop_status_;
  tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr mrm_emergency_stop_status_;

  void onHazardStatusStamped(
    const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);
  void onPrevControlCommand(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onControlMode(const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg);
  void onMrmComfortableStopStatus(
    const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg);
  void onMrmEmergencyStopStatus(
    const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    pub_control_command_;

  // rclcpp::Publisher<tier4_vehicle_msgs::msg::ShiftStamped>::SharedPtr pub_shift_;
  // rclcpp::Publisher<tier4_vehicle_msgs::msg::TurnSignal>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    pub_hazard_cmd_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_cmd_;

  autoware_auto_vehicle_msgs::msg::HazardLightsCommand createHazardCmdMsg();
  autoware_auto_vehicle_msgs::msg::GearCommand createGearCmdMsg();
  void publishControlCommands();

  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr pub_mrm_state_;

  autoware_adapi_v1_msgs::msg::MrmState mrm_state_;
  void publishMrmState();

  // Clients
  rclcpp::CallbackGroup::SharedPtr client_mrm_comfortable_stop_group_;
  rclcpp::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_comfortable_stop_;
  rclcpp::CallbackGroup::SharedPtr client_mrm_emergency_stop_group_;
  rclcpp::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_emergency_stop_;

  void callMrmBehavior(
    const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior) const;
  void cancelMrmBehavior(
    const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior) const;
  void logMrmCallingResult(
    const tier4_system_msgs::srv::OperateMrm::Response & result, const std::string & behavior,
    bool is_call) const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  Param param_;

  bool isDataReady();
  void onTimer();

  // Heartbeat
  rclcpp::Time stamp_hazard_status_;

  // Algorithm
  rclcpp::Time takeover_requested_time_;
  bool is_takeover_request_ = false;
  void transitionTo(const int new_state);
  void updateMrmState();
  void operateMrm();
  autoware_adapi_v1_msgs::msg::MrmState::_behavior_type getCurrentMrmBehavior();
  bool isStopped();
  bool isEmergency(const autoware_auto_system_msgs::msg::HazardStatus & hazard_status);
};

#endif  // EMERGENCY_HANDLER__EMERGENCY_HANDLER_CORE_HPP_
