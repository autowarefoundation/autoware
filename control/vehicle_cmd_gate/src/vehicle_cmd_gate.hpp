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

#ifndef VEHICLE_CMD_GATE_HPP_
#define VEHICLE_CMD_GATE_HPP_

#include "pause_interface.hpp"
#include "vehicle_cmd_filter.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <tier4_external_api_msgs/msg/emergency.hpp>
#include <tier4_external_api_msgs/msg/heartbeat.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <tier4_external_api_msgs/srv/set_emergency.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>

#include <memory>

namespace vehicle_cmd_gate
{

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using tier4_control_msgs::msg::GateMode;
using tier4_external_api_msgs::msg::Emergency;
using tier4_external_api_msgs::msg::Heartbeat;
using tier4_external_api_msgs::srv::SetEmergency;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_vehicle_msgs::msg::VehicleEmergencyStamped;

using diagnostic_msgs::msg::DiagnosticStatus;
using nav_msgs::msg::Odometry;

using EngageMsg = autoware_auto_vehicle_msgs::msg::Engage;
using EngageSrv = tier4_external_api_msgs::srv::Engage;

struct Commands
{
  AckermannControlCommand control;
  TurnIndicatorsCommand turn_indicator;
  HazardLightsCommand hazard_light;
  GearCommand gear;
  explicit Commands(const uint8_t & default_gear = GearCommand::PARK)
  {
    gear.command = default_gear;
  }
};

class VehicleCmdGate : public rclcpp::Node
{
public:
  explicit VehicleCmdGate(const rclcpp::NodeOptions & node_options);

private:
  // Publisher
  rclcpp::Publisher<VehicleEmergencyStamped>::SharedPtr vehicle_cmd_emergency_pub_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_cmd_pub_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_indicator_cmd_pub_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_light_cmd_pub_;
  rclcpp::Publisher<GateMode>::SharedPtr gate_mode_pub_;
  rclcpp::Publisher<EngageMsg>::SharedPtr engage_pub_;
  rclcpp::Publisher<OperationModeState>::SharedPtr operation_mode_pub_;

  // Subscription
  rclcpp::Subscription<Heartbeat>::SharedPtr external_emergency_stop_heartbeat_sub_;
  rclcpp::Subscription<GateMode>::SharedPtr gate_mode_sub_;
  rclcpp::Subscription<SteeringReport>::SharedPtr steer_sub_;
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_sub_;
  rclcpp::Subscription<MrmState>::SharedPtr mrm_state_sub_;

  void onGateMode(GateMode::ConstSharedPtr msg);
  void onExternalEmergencyStopHeartbeat(Heartbeat::ConstSharedPtr msg);
  void onSteering(SteeringReport::ConstSharedPtr msg);
  void onMrmState(MrmState::ConstSharedPtr msg);

  bool is_engaged_;
  bool is_system_emergency_ = false;
  bool is_external_emergency_stop_ = false;
  double current_steer_ = 0;
  GateMode current_gate_mode_;
  MrmState current_mrm_state_;

  // Heartbeat
  std::shared_ptr<rclcpp::Time> emergency_state_heartbeat_received_time_;
  bool is_emergency_state_heartbeat_timeout_ = false;
  std::shared_ptr<rclcpp::Time> external_emergency_stop_heartbeat_received_time_;
  bool is_external_emergency_stop_heartbeat_timeout_ = false;
  bool isHeartbeatTimeout(
    const std::shared_ptr<rclcpp::Time> & heartbeat_received_time, const double timeout);

  // Check initialization
  bool isDataReady();

  // Subscriber for auto
  Commands auto_commands_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr auto_control_cmd_sub_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr auto_turn_indicator_cmd_sub_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr auto_hazard_light_cmd_sub_;
  rclcpp::Subscription<GearCommand>::SharedPtr auto_gear_cmd_sub_;
  void onAutoCtrlCmd(AckermannControlCommand::ConstSharedPtr msg);
  void onAutoTurnIndicatorsCmd(TurnIndicatorsCommand::ConstSharedPtr msg);
  void onAutoHazardLightsCmd(HazardLightsCommand::ConstSharedPtr msg);
  void onAutoShiftCmd(GearCommand::ConstSharedPtr msg);

  // Subscription for external
  Commands remote_commands_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr remote_control_cmd_sub_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr remote_turn_indicator_cmd_sub_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr remote_hazard_light_cmd_sub_;
  rclcpp::Subscription<GearCommand>::SharedPtr remote_gear_cmd_sub_;
  void onRemoteCtrlCmd(AckermannControlCommand::ConstSharedPtr msg);
  void onRemoteTurnIndicatorsCmd(TurnIndicatorsCommand::ConstSharedPtr msg);
  void onRemoteHazardLightsCmd(HazardLightsCommand::ConstSharedPtr msg);
  void onRemoteShiftCmd(GearCommand::ConstSharedPtr msg);

  // Subscription for emergency
  Commands emergency_commands_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr emergency_control_cmd_sub_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr emergency_hazard_light_cmd_sub_;
  rclcpp::Subscription<GearCommand>::SharedPtr emergency_gear_cmd_sub_;
  void onEmergencyCtrlCmd(AckermannControlCommand::ConstSharedPtr msg);
  void onEmergencyTurnIndicatorsCmd(TurnIndicatorsCommand::ConstSharedPtr msg);
  void onEmergencyHazardLightsCmd(HazardLightsCommand::ConstSharedPtr msg);
  void onEmergencyShiftCmd(GearCommand::ConstSharedPtr msg);

  // Parameter
  double update_period_;
  bool use_emergency_handling_;
  bool check_external_emergency_heartbeat_;
  double system_emergency_heartbeat_timeout_;
  double external_emergency_stop_heartbeat_timeout_;
  double stop_hold_acceleration_;
  double emergency_acceleration_;

  // Service
  rclcpp::Service<tier4_external_api_msgs::srv::Engage>::SharedPtr srv_engage_;
  rclcpp::Service<tier4_external_api_msgs::srv::SetEmergency>::SharedPtr srv_external_emergency_;
  rclcpp::Publisher<Emergency>::SharedPtr pub_external_emergency_;
  void onEngageService(
    const tier4_external_api_msgs::srv::Engage::Request::SharedPtr request,
    const tier4_external_api_msgs::srv::Engage::Response::SharedPtr response);
  void onExternalEmergencyStopService(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<tier4_external_api_msgs::srv::SetEmergency::Request> request,
    const std::shared_ptr<tier4_external_api_msgs::srv::SetEmergency::Response> response);

  // TODO(Takagi, Isamu): deprecated
  rclcpp::Subscription<EngageMsg>::SharedPtr engage_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_external_emergency_stop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_external_emergency_stop_;
  void onEngage(EngageMsg::ConstSharedPtr msg);
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
  rclcpp::TimerBase::SharedPtr timer_pub_status_;

  void onTimer();
  void publishControlCommands(const Commands & input_msg);
  void publishEmergencyStopControlCommands();
  void publishStatus();

  // Diagnostics Updater
  diagnostic_updater::Updater updater_;

  void checkExternalEmergencyStop(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Algorithm
  AckermannControlCommand prev_control_cmd_;
  AckermannControlCommand createStopControlCmd() const;
  AckermannControlCommand createEmergencyStopControlCmd() const;

  std::shared_ptr<rclcpp::Time> prev_time_;
  double getDt();

  VehicleCmdFilter filter_;
  AckermannControlCommand filterControlCommand(const AckermannControlCommand & msg);

  // filtering on transition
  OperationModeState current_operation_mode_;
  VehicleCmdFilter filter_on_transition_;

  // Pause interface for API
  std::unique_ptr<PauseInterface> pause_;
};

}  // namespace vehicle_cmd_gate
#endif  // VEHICLE_CMD_GATE_HPP_
