// Copyright 2024 TIER IV, Inc.
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

#ifndef MRM_HANDLER__MRM_HANDLER_CORE_HPP_
#define MRM_HANDLER__MRM_HANDLER_CORE_HPP_

// Core
#include <memory>
#include <optional>
#include <string>
#include <variant>

// Autoware
#include <autoware/universe_utils/ros/polling_subscriber.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/msg/operation_mode_availability.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS 2 core
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
  double timeout_operation_mode_availability;
  double timeout_call_mrm_behavior;
  double timeout_cancel_mrm_behavior;
  bool use_emergency_holding;
  double timeout_emergency_recovery;
  bool use_parking_after_stopped;
  bool use_pull_over;
  bool use_comfortable_stop;
  HazardLampPolicy turning_hazard_on{};
};

class MrmHandler : public rclcpp::Node
{
public:
  explicit MrmHandler(const rclcpp::NodeOptions & options);

private:
  // type
  enum RequestType { CALL, CANCEL };

  // Subscribers with callback
  rclcpp::Subscription<tier4_system_msgs::msg::OperationModeAvailability>::SharedPtr
    sub_operation_mode_availability_;
  // Subscribers without callback
  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odom_{
    this, "~/input/odometry"};
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_vehicle_msgs::msg::ControlModeReport>
    sub_control_mode_{this, "~/input/control_mode"};
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_system_msgs::msg::MrmBehaviorStatus>
    sub_mrm_pull_over_status_{this, "~/input/mrm/pull_over/status"};
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_system_msgs::msg::MrmBehaviorStatus>
    sub_mrm_comfortable_stop_status_{this, "~/input/mrm/comfortable_stop/status"};
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_system_msgs::msg::MrmBehaviorStatus>
    sub_mrm_emergency_stop_status_{this, "~/input/mrm/emergency_stop/status"};
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_adapi_v1_msgs::msg::OperationModeState>
    sub_operation_mode_state_{this, "~/input/api/operation_mode/state"};
  autoware::universe_utils::InterProcessPollingSubscriber<autoware_vehicle_msgs::msg::GearCommand>
    sub_gear_cmd_{this, "~/input/gear"};

  tier4_system_msgs::msg::OperationModeAvailability::ConstSharedPtr operation_mode_availability_;

  void onOperationModeAvailability(
    const tier4_system_msgs::msg::OperationModeAvailability::ConstSharedPtr msg);

  // Publisher

  // rclcpp::Publisher<tier4_vehicle_msgs::msg::ShiftStamped>::SharedPtr pub_shift_;
  // rclcpp::Publisher<tier4_vehicle_msgs::msg::TurnSignal>::SharedPtr pub_turn_signal_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr pub_hazard_cmd_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr pub_gear_cmd_;

  void publishHazardCmd();
  void publishGearCmd();

  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::MrmState>::SharedPtr pub_mrm_state_;

  autoware_adapi_v1_msgs::msg::MrmState mrm_state_;
  void publishMrmState();

  // Clients
  rclcpp::CallbackGroup::SharedPtr client_mrm_pull_over_group_;
  rclcpp::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_pull_over_;
  rclcpp::CallbackGroup::SharedPtr client_mrm_comfortable_stop_group_;
  rclcpp::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_comfortable_stop_;
  rclcpp::CallbackGroup::SharedPtr client_mrm_emergency_stop_group_;
  rclcpp::Client<tier4_system_msgs::srv::OperateMrm>::SharedPtr client_mrm_emergency_stop_;

  bool requestMrmBehavior(
    const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior,
    RequestType request_type) const;
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
  rclcpp::Time stamp_operation_mode_availability_;
  std::optional<rclcpp::Time> stamp_autonomous_become_unavailable_ = std::nullopt;
  bool is_operation_mode_availability_timeout;
  void checkOperationModeAvailabilityTimeout();

  // Algorithm
  bool is_emergency_holding_ = false;
  uint8_t last_gear_command_{autoware_vehicle_msgs::msg::GearCommand::DRIVE};
  void transitionTo(const int new_state);
  void updateMrmState();
  void operateMrm();
  void handleFailedRequest();
  autoware_adapi_v1_msgs::msg::MrmState::_behavior_type getCurrentMrmBehavior();
  bool isStopped();
  bool isEmergency() const;
  bool isControlModeAutonomous();
  bool isOperationModeAutonomous();
  bool isPullOverStatusAvailable();
  bool isComfortableStopStatusAvailable();
  bool isEmergencyStopStatusAvailable();
  bool isArrivedAtGoal();
};

#endif  // MRM_HANDLER__MRM_HANDLER_CORE_HPP_
