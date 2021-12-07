// Copyright 2015-2019 Autoware Foundation
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

#ifndef VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_
#define VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_

#include "vehicle_cmd_gate/vehicle_cmd_filter.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_control_msgs/msg/gate_mode.hpp>
#include <autoware_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_external_api_msgs/msg/emergency.hpp>
#include <autoware_external_api_msgs/msg/heartbeat.hpp>
#include <autoware_external_api_msgs/srv/engage.hpp>
#include <autoware_external_api_msgs/srv/set_emergency.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>

struct Commands
{
  autoware_auto_control_msgs::msg::AckermannControlCommand control;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand turn_indicator;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand hazard_light;
  autoware_auto_vehicle_msgs::msg::GearCommand gear;
  explicit Commands(
    const uint8_t & default_gear = autoware_auto_vehicle_msgs::msg::GearCommand::PARK)
  {
    gear.command = default_gear;
  }
};

class VehicleCmdGate : public rclcpp::Node
{
  using VehicleEmergencyStamped = autoware_vehicle_msgs::msg::VehicleEmergencyStamped;

public:
  explicit VehicleCmdGate(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  rclcpp::Publisher<VehicleEmergencyStamped>::SharedPtr vehicle_cmd_emergency_pub_;
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    control_cmd_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicator_cmd_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_light_cmd_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::GateMode>::SharedPtr gate_mode_pub_;
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_pub_;

  // Subscription
  rclcpp::Subscription<autoware_auto_system_msgs::msg::EmergencyState>::SharedPtr
    emergency_state_sub_;
  rclcpp::Subscription<autoware_external_api_msgs::msg::Heartbeat>::SharedPtr
    external_emergency_stop_heartbeat_sub_;
  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr gate_mode_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr steer_sub_;

  void onGateMode(autoware_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onEmergencyState(autoware_auto_system_msgs::msg::EmergencyState::ConstSharedPtr msg);
  void onExternalEmergencyStopHeartbeat(
    autoware_external_api_msgs::msg::Heartbeat::ConstSharedPtr msg);
  void onSteering(autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg);

  bool is_engaged_;
  bool is_system_emergency_ = false;
  bool is_external_emergency_stop_ = false;
  double current_steer_ = 0;
  autoware_control_msgs::msg::GateMode current_gate_mode_;

  // Heartbeat
  std::shared_ptr<rclcpp::Time> emergency_state_heartbeat_received_time_;
  bool is_emergency_state_heartbeat_timeout_ = false;
  std::shared_ptr<rclcpp::Time> external_emergency_stop_heartbeat_received_time_;
  bool is_external_emergency_stop_heartbeat_timeout_ = false;
  bool isHeartbeatTimeout(
    const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout);

  // Subscriber for auto
  Commands auto_commands_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    auto_control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    auto_turn_indicator_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    auto_hazard_light_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr auto_gear_cmd_sub_;
  void onAutoCtrlCmd(autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onAutoTurnIndicatorsCmd(
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void onAutoHazardLightsCmd(
    autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  void onAutoShiftCmd(autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);

  // Subscription for external
  Commands remote_commands_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    remote_control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    remote_turn_indicator_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    remote_hazard_light_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr
    remote_gear_cmd_sub_;
  void onRemoteCtrlCmd(
    autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onRemoteTurnIndicatorsCmd(
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void onRemoteHazardLightsCmd(
    autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  void onRemoteShiftCmd(autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);

  // Subscription for emergency
  Commands emergency_commands_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    emergency_control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    emergency_hazard_light_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr
    emergency_gear_cmd_sub_;
  void onEmergencyCtrlCmd(
    autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onEmergencyTurnIndicatorsCmd(
    autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
  void onEmergencyHazardLightsCmd(
    autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ConstSharedPtr msg);
  void onEmergencyShiftCmd(autoware_auto_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);

  // Parameter
  double update_period_;
  bool use_emergency_handling_;
  bool use_external_emergency_stop_;
  double system_emergency_heartbeat_timeout_;
  double external_emergency_stop_heartbeat_timeout_;
  double stop_hold_acceleration_;
  double emergency_acceleration_;

  // Service
  rclcpp::Service<autoware_external_api_msgs::srv::Engage>::SharedPtr srv_engage_;
  rclcpp::Service<autoware_external_api_msgs::srv::SetEmergency>::SharedPtr srv_external_emergency_;
  rclcpp::Publisher<autoware_external_api_msgs::msg::Emergency>::SharedPtr pub_external_emergency_;
  void onEngageService(
    const autoware_external_api_msgs::srv::Engage::Request::SharedPtr request,
    const autoware_external_api_msgs::srv::Engage::Response::SharedPtr response);
  void onExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<autoware_external_api_msgs::srv::SetEmergency::Request> request,
    const std::shared_ptr<autoware_external_api_msgs::srv::SetEmergency::Response> response);

  // TODO(Takagi, Isamu): deprecated
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_external_emergency_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_external_emergency_stop_;
  void onEngage(autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
  bool onSetExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  bool onClearExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> req_header,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Timer / Event
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();
  void publishControlCommands(const Commands & input_msg);
  void publishEmergencyStopControlCommands();

  // Diagnostics Updater
  diagnostic_updater::Updater updater_;

  void checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Algorithm
  autoware_auto_control_msgs::msg::AckermannControlCommand prev_control_cmd_;
  autoware_auto_control_msgs::msg::AckermannControlCommand createStopControlCmd() const;
  autoware_auto_control_msgs::msg::AckermannControlCommand createEmergencyStopControlCmd() const;

  std::shared_ptr<rclcpp::Time> prev_time_;
  double getDt();

  VehicleCmdFilter filter_;
  autoware_auto_control_msgs::msg::AckermannControlCommand filterControlCommand(
    const autoware_auto_control_msgs::msg::AckermannControlCommand & msg);

  // Start request service
  struct StartRequest
  {
  private:
    static constexpr double eps = 1e-3;
    using ControlCommandStamped = autoware_auto_control_msgs::msg::AckermannControlCommand;

  public:
    StartRequest(rclcpp::Node * node, bool use_start_request);
    bool isAccepted();
    void publishStartAccepted();
    void checkStopped(const ControlCommandStamped & control);
    void checkStartRequest(const ControlCommandStamped & control);

  private:
    bool use_start_request_;
    bool is_start_requesting_;
    bool is_start_accepted_;
    bool is_start_cancelled_;
    nav_msgs::msg::Odometry current_twist_;

    rclcpp::Node * node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr request_start_cli_;
    rclcpp::Publisher<autoware_debug_msgs::msg::BoolStamped>::SharedPtr request_start_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_twist_sub_;
    void onCurrentTwist(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  };

  std::unique_ptr<StartRequest> start_request_;
};

#endif  // VEHICLE_CMD_GATE__VEHICLE_CMD_GATE_HPP_
