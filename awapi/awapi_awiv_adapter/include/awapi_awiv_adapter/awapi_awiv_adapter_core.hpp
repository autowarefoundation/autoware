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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_AWIV_ADAPTER_CORE_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_AWIV_ADAPTER_CORE_HPP_

#include "awapi_awiv_adapter/awapi_autoware_state_publisher.hpp"
#include "awapi_awiv_adapter/awapi_autoware_util.hpp"
#include "awapi_awiv_adapter/awapi_lane_change_state_publisher.hpp"
#include "awapi_awiv_adapter/awapi_max_velocity_publisher.hpp"
#include "awapi_awiv_adapter/awapi_obstacle_avoidance_state_publisher.hpp"
#include "awapi_awiv_adapter/awapi_pacmod_util.hpp"
#include "awapi_awiv_adapter/awapi_stop_reason_aggregator.hpp"
#include "awapi_awiv_adapter/awapi_v2x_aggregator.hpp"
#include "awapi_awiv_adapter/awapi_vehicle_state_publisher.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_control_msgs/msg/gate_mode.hpp>
#include <autoware_planning_msgs/msg/stop_reason_array.hpp>
#include <autoware_v2x_msgs/msg/infrastructure_command_array.hpp>
#include <autoware_v2x_msgs/msg/virtual_traffic_light_state_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace autoware_api
{
class AutowareIvAdapter : public rclcpp::Node
{
public:
  AutowareIvAdapter();

private:
  // subscriber
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steer_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_vehicle_cmd_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    sub_turn_indicators_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsReport>::SharedPtr
    sub_hazard_lights_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr sub_gear_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::BatteryStatus>::SharedPtr sub_battery_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_nav_sat_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr
    sub_autoware_state_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    sub_control_mode_;
  rclcpp::Subscription<autoware_control_msgs::msg::GateMode>::SharedPtr sub_gate_mode_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::EmergencyState>::SharedPtr sub_emergency_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::HazardStatusStamped>::SharedPtr
    sub_hazard_status_;
  rclcpp::Subscription<autoware_planning_msgs::msg::StopReasonArray>::SharedPtr sub_stop_reason_;
  rclcpp::Subscription<autoware_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr
    sub_v2x_command_;
  rclcpp::Subscription<autoware_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr
    sub_v2x_state_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diagnostics_;
  rclcpp::Subscription<pacmod3_msgs::msg::GlobalRpt>::SharedPtr sub_global_rpt_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneChangeStatus>::SharedPtr
    sub_lane_change_available_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneChangeStatus>::SharedPtr
    sub_lane_change_ready_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr
    sub_lane_change_candidate_;
  rclcpp::Subscription<autoware_planning_msgs::msg::IsAvoidancePossible>::SharedPtr
    sub_obstacle_avoid_ready_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    sub_obstacle_avoid_candidate_;
  rclcpp::Subscription<autoware_api_msgs::msg::VelocityLimit>::SharedPtr sub_max_velocity_;
  rclcpp::Subscription<autoware_planning_msgs::msg::VelocityLimit>::SharedPtr
    sub_current_max_velocity_;
  rclcpp::Subscription<autoware_api_msgs::msg::StopCommand>::SharedPtr sub_temporary_stop_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_autoware_traj_;
  rclcpp::Subscription<autoware_api_msgs::msg::DoorControlCommand>::SharedPtr sub_door_control_;
  rclcpp::Subscription<pacmod3_msgs::msg::SystemRptInt>::SharedPtr sub_door_status_;

  // publisher
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr pub_door_control_;
  rclcpp::Publisher<autoware_api_msgs::msg::DoorStatus>::SharedPtr pub_door_status_;
  rclcpp::Publisher<autoware_v2x_msgs::msg::InfrastructureCommandArray>::SharedPtr pub_v2x_command_;
  rclcpp::Publisher<autoware_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr
    pub_v2x_state_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // callback function
  void callbackSteer(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg_ptr);
  void callbackVehicleCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg_ptr);
  void callbackTurnIndicators(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr msg_ptr);
  void callbackHazardLights(
    const autoware_auto_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr msg_ptr);
  void callbackTwist(const nav_msgs::msg::Odometry::ConstSharedPtr msg_ptr);
  void callbackGear(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr msg_ptr);
  void callbackBattery(const autoware_vehicle_msgs::msg::BatteryStatus::ConstSharedPtr msg_ptr);
  void callbackNavSat(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg_ptr);
  void callbackAutowareState(
    const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg_ptr);
  void callbackControlMode(
    const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg_ptr);
  void callbackGateMode(const autoware_control_msgs::msg::GateMode::ConstSharedPtr msg_ptr);
  void callbackEmergencyState(
    const autoware_auto_system_msgs::msg::EmergencyState::ConstSharedPtr msg_ptr);
  void callbackHazardStatus(
    const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg_ptr);
  void callbackStopReason(
    const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr msg_ptr);
  void callbackV2XCommand(
    const autoware_v2x_msgs::msg::InfrastructureCommandArray::ConstSharedPtr msg_ptr);
  void callbackV2XState(
    const autoware_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr msg_ptr);
  void callbackDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg_ptr);
  void callbackGlobalRpt(const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr msg_ptr);
  void callbackLaneChangeAvailable(
    const autoware_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr msg_ptr);
  void callbackLaneChangeReady(
    const autoware_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr msg_ptr);
  void callbackLaneChangeCandidatePath(
    const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr msg_ptr);
  void callbackLaneObstacleAvoidReady(
    const autoware_planning_msgs::msg::IsAvoidancePossible::ConstSharedPtr msg_ptr);
  void callbackLaneObstacleAvoidCandidatePath(
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr);
  void callbackMaxVelocity(const autoware_api_msgs::msg::VelocityLimit::ConstSharedPtr msg_ptr);
  void callbackCurrentMaxVelocity(
    const autoware_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg_ptr);
  void callbackTemporaryStop(const autoware_api_msgs::msg::StopCommand::ConstSharedPtr msg_ptr);
  void callbackAutowareTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr);
  void callbackDoorControl(
    const autoware_api_msgs::msg::DoorControlCommand::ConstSharedPtr msg_ptr);
  void callbackDoorStatus(const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr msg_ptr);

  // timer function
  void timerCallback();

  void emergencyParamCheck(const bool emergency_stop_param);
  void getCurrentPose();

  // parameter
  AutowareInfo aw_info_;
  std::unique_ptr<AutowareIvVehicleStatePublisher> vehicle_state_publisher_;
  std::unique_ptr<AutowareIvAutowareStatePublisher> autoware_state_publisher_;
  std::unique_ptr<AutowareIvStopReasonAggregator> stop_reason_aggregator_;
  std::unique_ptr<AutowareIvV2XAggregator> v2x_aggregator_;
  std::unique_ptr<AutowareIvLaneChangeStatePublisher> lane_change_state_publisher_;
  std::unique_ptr<AutowareIvObstacleAvoidanceStatePublisher> obstacle_avoidance_state_publisher_;
  std::unique_ptr<AutowareIvMaxVelocityPublisher> max_velocity_publisher_;
  double status_pub_hz_;
  double stop_reason_timeout_;
  double stop_reason_thresh_dist_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_AWIV_ADAPTER_CORE_HPP_
