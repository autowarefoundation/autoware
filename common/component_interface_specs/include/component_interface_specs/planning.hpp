// Copyright 2022 TIER IV, Inc.
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

#ifndef COMPONENT_INTERFACE_SPECS__PLANNING_HPP_
#define COMPONENT_INTERFACE_SPECS__PLANNING_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

namespace planning_interface
{

struct SetRoutePoints
{
  using Service = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  static constexpr char name[] = "/planning/mission_planning/set_route_points";
};

struct SetRoute
{
  using Service = autoware_adapi_v1_msgs::srv::SetRoute;
  static constexpr char name[] = "/planning/mission_planning/set_route";
};

struct ChangeRoutePoints
{
  using Service = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  static constexpr char name[] = "/planning/mission_planning/change_route_points";
};

struct ChangeRoute
{
  using Service = autoware_adapi_v1_msgs::srv::SetRoute;
  static constexpr char name[] = "/planning/mission_planning/change_route";
};

struct ClearRoute
{
  using Service = autoware_adapi_v1_msgs::srv::ClearRoute;
  static constexpr char name[] = "/planning/mission_planning/clear_route";
};

struct RouteState
{
  using Message = autoware_adapi_v1_msgs::msg::RouteState;
  static constexpr char name[] = "/planning/mission_planning/route_state";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct Route
{
  using Message = autoware_planning_msgs::msg::LaneletRoute;
  static constexpr char name[] = "/planning/mission_planning/route";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

struct Trajectory
{
  using Message = autoware_auto_planning_msgs::msg::Trajectory;
  static constexpr char name[] = "/planning/scenario_planning/trajectory";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace planning_interface

#endif  // COMPONENT_INTERFACE_SPECS__PLANNING_HPP_
