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

#ifndef TIER4_AUTO_MSGS_CONVERTER__TIER4_AUTO_MSGS_CONVERTER_HPP_
#define TIER4_AUTO_MSGS_CONVERTER__TIER4_AUTO_MSGS_CONVERTER_HPP_

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_system_msgs/msg/autoware_state.hpp"
#include "autoware_auto_system_msgs/msg/hazard_status_stamped.hpp"
#include "autoware_auto_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp"
#include "tier4_external_api_msgs/msg/gear_shift_stamped.hpp"
#include "tier4_external_api_msgs/msg/turn_signal_stamped.hpp"
#include "tier4_planning_msgs/msg/path.hpp"
#include "tier4_planning_msgs/msg/trajectory.hpp"
#include "tier4_system_msgs/msg/autoware_state.hpp"
#include "tier4_system_msgs/msg/hazard_status_stamped.hpp"
#include "tier4_vehicle_msgs/msg/control_mode.hpp"
#include "tier4_vehicle_msgs/msg/shift_stamped.hpp"
#include "tier4_vehicle_msgs/msg/steering.hpp"
#include "tier4_vehicle_msgs/msg/turn_signal.hpp"

namespace tier4_auto_msgs_converter
{
struct LightSignal
{
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand turn_signal;
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand hazard_signal;
};

inline auto convert(const autoware_auto_system_msgs::msg::AutowareState & state)
{
  tier4_system_msgs::msg::AutowareState iv_state;
  switch (state.state) {
    case autoware_auto_system_msgs::msg::AutowareState::INITIALIZING:
      iv_state.state = tier4_system_msgs::msg::AutowareState::INITIALIZING_VEHICLE;
      break;
    case autoware_auto_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE:
      iv_state.state = tier4_system_msgs::msg::AutowareState::WAITING_FOR_ROUTE;
      break;
    case autoware_auto_system_msgs::msg::AutowareState::PLANNING:
      iv_state.state = tier4_system_msgs::msg::AutowareState::PLANNING;
      break;
    case autoware_auto_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE:
      iv_state.state = tier4_system_msgs::msg::AutowareState::WAITING_FOR_ENGAGE;
      break;
    case autoware_auto_system_msgs::msg::AutowareState::DRIVING:
      iv_state.state = tier4_system_msgs::msg::AutowareState::DRIVING;
      break;
    case autoware_auto_system_msgs::msg::AutowareState::ARRIVED_GOAL:
      iv_state.state = tier4_system_msgs::msg::AutowareState::ARRIVAL_GOAL;
      break;
    case autoware_auto_system_msgs::msg::AutowareState::FINALIZING:
      iv_state.state = tier4_system_msgs::msg::AutowareState::FINALIZING;
      break;
  }
  return iv_state;
}

inline auto convert(const autoware_auto_system_msgs::msg::HazardStatusStamped & status)
{
  tier4_system_msgs::msg::HazardStatusStamped iv_status;
  iv_status.header.stamp = status.stamp;
  iv_status.status.level = status.status.level;
  iv_status.status.emergency = status.status.emergency;
  iv_status.status.emergency_holding = status.status.emergency_holding;
  iv_status.status.diagnostics_nf = status.status.diag_no_fault;
  iv_status.status.diagnostics_sf = status.status.diag_safe_fault;
  iv_status.status.diagnostics_lf = status.status.diag_latent_fault;
  iv_status.status.diagnostics_spf = status.status.diag_single_point_fault;
  return iv_status;
}

inline auto convert(const autoware_auto_planning_msgs::msg::Path & path)
{
  tier4_planning_msgs::msg::Path iv_path;
  iv_path.header = path.header;
  iv_path.drivable_area = path.drivable_area;
  iv_path.points.reserve(path.points.size());
  for (const auto point : path.points) {
    tier4_planning_msgs::msg::PathPoint iv_point;
    iv_point.pose = point.pose;
    iv_point.twist.linear.x = point.longitudinal_velocity_mps;
    iv_point.twist.linear.y = point.lateral_velocity_mps;
    iv_point.twist.angular.z = point.heading_rate_rps;
    iv_point.type = 0;  // not used
    iv_path.points.push_back(iv_point);
  }
  return iv_path;
}

inline auto convert(const autoware_auto_planning_msgs::msg::Trajectory & traj)
{
  tier4_planning_msgs::msg::Trajectory iv_traj;
  iv_traj.header = traj.header;
  iv_traj.points.reserve(traj.points.size());
  for (const auto point : traj.points) {
    tier4_planning_msgs::msg::TrajectoryPoint iv_point;
    iv_point.pose = point.pose;
    iv_point.accel.linear.x = point.acceleration_mps2;
    iv_point.twist.linear.x = point.longitudinal_velocity_mps;
    iv_point.twist.linear.y = point.lateral_velocity_mps;
    iv_point.twist.angular.z = point.heading_rate_rps;
    iv_traj.points.push_back(iv_point);
  }
  return iv_traj;
}

inline auto convert(const autoware_auto_vehicle_msgs::msg::ControlModeReport & mode)
{
  tier4_vehicle_msgs::msg::ControlMode iv_mode;
  iv_mode.header.stamp = mode.stamp;
  switch (mode.mode) {
    case autoware_auto_vehicle_msgs::msg::ControlModeReport::MANUAL:
      iv_mode.data = tier4_vehicle_msgs::msg::ControlMode::MANUAL;
      break;
    case autoware_auto_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS:
      iv_mode.data = tier4_vehicle_msgs::msg::ControlMode::AUTO;
      break;
    default:
      iv_mode.data = tier4_vehicle_msgs::msg::ControlMode::MANUAL;
      break;
  }
  return iv_mode;
}

inline auto convert(const autoware_auto_vehicle_msgs::msg::GearReport & gear)
{
  tier4_vehicle_msgs::msg::ShiftStamped iv_shift;
  iv_shift.header.stamp = gear.stamp;
  switch (gear.report) {
    case autoware_auto_vehicle_msgs::msg::GearReport::PARK:
      iv_shift.shift.data = tier4_vehicle_msgs::msg::Shift::PARKING;
      break;
    case autoware_auto_vehicle_msgs::msg::GearReport::REVERSE:
    case autoware_auto_vehicle_msgs::msg::GearReport::REVERSE_2:
      iv_shift.shift.data = tier4_vehicle_msgs::msg::Shift::REVERSE;
      break;
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_2:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_3:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_4:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_5:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_6:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_7:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_8:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_9:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_10:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_11:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_12:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_13:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_14:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_15:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_16:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_17:
    case autoware_auto_vehicle_msgs::msg::GearReport::DRIVE_18:
      iv_shift.shift.data = tier4_vehicle_msgs::msg::Shift::DRIVE;
      break;
    case autoware_auto_vehicle_msgs::msg::GearReport::LOW:
    case autoware_auto_vehicle_msgs::msg::GearReport::LOW_2:
      iv_shift.shift.data = tier4_vehicle_msgs::msg::Shift::LOW;
      break;
    default:
      iv_shift.shift.data = tier4_vehicle_msgs::msg::Shift::NONE;
      break;
  }
  return iv_shift;
}

inline auto convert(const tier4_external_api_msgs::msg::GearShiftStamped & shift)
{
  autoware_auto_vehicle_msgs::msg::GearCommand auto_gear;
  auto_gear.stamp = shift.stamp;
  switch (shift.gear_shift.data) {
    case tier4_external_api_msgs::msg::GearShift::PARKING:
      auto_gear.command = autoware_auto_vehicle_msgs::msg::GearCommand::PARK;
      break;
    case tier4_external_api_msgs::msg::GearShift::REVERSE:
      auto_gear.command = autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE;
      break;
    case tier4_external_api_msgs::msg::GearShift::DRIVE:
      auto_gear.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;
      break;
    case tier4_external_api_msgs::msg::GearShift::LOW:
      auto_gear.command = autoware_auto_vehicle_msgs::msg::GearCommand::LOW;
      break;
    default:
      constexpr int default_val = 0;
      auto_gear.command = default_val;
      break;
  }
  return auto_gear;
}

inline auto convert(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport & turn_indicators,
  const autoware_auto_vehicle_msgs::msg::HazardLightsReport & hazard_lights)
{
  tier4_vehicle_msgs::msg::TurnSignal iv_turn_signal;

  if (hazard_lights.report == autoware_auto_vehicle_msgs::msg::HazardLightsReport::ENABLE) {
    iv_turn_signal.header.stamp = hazard_lights.stamp;
    iv_turn_signal.data = tier4_vehicle_msgs::msg::TurnSignal::HAZARD;
    return iv_turn_signal;
  }

  iv_turn_signal.header.stamp = turn_indicators.stamp;
  switch (turn_indicators.report) {
    case autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT:
      iv_turn_signal.data = tier4_vehicle_msgs::msg::TurnSignal::LEFT;
      break;
    case autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT:
      iv_turn_signal.data = tier4_vehicle_msgs::msg::TurnSignal::RIGHT;
      break;
    default:
      iv_turn_signal.data = tier4_vehicle_msgs::msg::TurnSignal::NONE;
      break;
  }
  return iv_turn_signal;
}

inline auto convert(const tier4_external_api_msgs::msg::TurnSignalStamped & in_signal)
{
  autoware_auto_vehicle_msgs::msg::HazardLightsCommand hazard;
  hazard.stamp = in_signal.stamp;
  autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand turn;
  turn.stamp = in_signal.stamp;

  switch (in_signal.turn_signal.data) {
    case tier4_vehicle_msgs::msg::TurnSignal::HAZARD:
      hazard.command = autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE;
      turn.command = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE;
      break;
    case tier4_vehicle_msgs::msg::TurnSignal::LEFT:
      hazard.command = autoware_auto_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
      turn.command = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT;
      break;
    case tier4_vehicle_msgs::msg::TurnSignal::RIGHT:
      hazard.command = autoware_auto_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
      turn.command = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT;
      break;
    default:
      hazard.command = autoware_auto_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
      turn.command = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::DISABLE;
  }

  LightSignal light_signal;
  light_signal.turn_signal = turn;
  light_signal.hazard_signal = hazard;
  return light_signal;
}

inline auto convert(const autoware_auto_vehicle_msgs::msg::SteeringReport & steering)
{
  tier4_vehicle_msgs::msg::Steering iv_steering;
  iv_steering.header.stamp = steering.stamp;
  iv_steering.data = steering.steering_tire_angle;
  return iv_steering;
}

}  // namespace tier4_auto_msgs_converter

#endif  // TIER4_AUTO_MSGS_CONVERTER__TIER4_AUTO_MSGS_CONVERTER_HPP_
