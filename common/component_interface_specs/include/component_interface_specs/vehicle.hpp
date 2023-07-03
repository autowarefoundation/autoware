// Copyright 2023 TIER IV, Inc.
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

#ifndef COMPONENT_INTERFACE_SPECS__VEHICLE_HPP_
#define COMPONENT_INTERFACE_SPECS__VEHICLE_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <tier4_vehicle_msgs/msg/battery_status.hpp>

namespace vehicle_interface
{

struct SteeringStatus
{
  using Message = autoware_auto_vehicle_msgs::msg::SteeringReport;
  static constexpr char name[] = "/vehicle/status/steering_status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct GearStatus
{
  using Message = autoware_auto_vehicle_msgs::msg::GearReport;
  static constexpr char name[] = "/vehicle/status/gear_status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct TurnIndicatorStatus
{
  using Message = autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport;
  static constexpr char name[] = "/vehicle/status/turn_indicators_status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct HazardLightStatus
{
  using Message = autoware_auto_vehicle_msgs::msg::HazardLightsReport;
  static constexpr char name[] = "/vehicle/status/hazard_lights_status";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

struct EnergyStatus
{
  using Message = tier4_vehicle_msgs::msg::BatteryStatus;
  static constexpr char name[] = "/vehicle/status/battery_charge";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace vehicle_interface

#endif  // COMPONENT_INTERFACE_SPECS__VEHICLE_HPP_
