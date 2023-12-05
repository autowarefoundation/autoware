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

#ifndef MISSION_PLANNER__MISSION_PLANNER_INTERFACE_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_INTERFACE_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mission_planner
{

struct SetMrmRoute
{
  using Service = autoware_adapi_v1_msgs::srv::SetRoutePoints;
  static constexpr char name[] = "~/srv/set_mrm_route";
};

struct ClearMrmRoute
{
  using Service = autoware_adapi_v1_msgs::srv::ClearRoute;
  static constexpr char name[] = "~/srv/clear_mrm_route";
};

struct ModifiedGoal
{
  using Message = autoware_planning_msgs::msg::PoseWithUuidStamped;
  static constexpr char name[] = "input/modified_goal";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_INTERFACE_HPP_
