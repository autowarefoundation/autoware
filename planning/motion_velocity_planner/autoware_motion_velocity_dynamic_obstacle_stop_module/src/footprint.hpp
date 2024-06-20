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

#ifndef FOOTPRINT_HPP_
#define FOOTPRINT_HPP_

#include "types.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
/// @brief create the footprint of the given obstacles and their projection over a fixed time
/// horizon
/// @param [in] obstacles obstacles
/// @param [in] params parameters used to create the footprint
/// @param [in] hysteresis [m] extra lateral distance to add to the footprints
/// @return forward footprint of the obstacle
autoware::universe_utils::MultiPolygon2d make_forward_footprints(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & obstacles,
  const PlannerParam & params, const double hysteresis);
/// @brief create the footprint of the given obstacle and its projection over a fixed time horizon
/// @param [in] obstacle obstacle
/// @param [in] params parameters used to create the footprint
/// @param [in] hysteresis [m] extra lateral distance to add to the footprint
/// @return forward footprint of the obstacle
autoware::universe_utils::Polygon2d make_forward_footprint(
  const autoware_perception_msgs::msg::PredictedObject & obstacle, const PlannerParam & params,
  const double hysteresis);
/// @brief project a footprint to the given pose
/// @param [in] base_footprint footprint to project
/// @param [in] pose projection pose
/// @return footprint projected to the given pose
autoware::universe_utils::Polygon2d project_to_pose(
  const autoware::universe_utils::Polygon2d & base_footprint,
  const geometry_msgs::msg::Pose & pose);
/// @brief create the rtree indexing the ego footprint along the trajectory
/// @param [inout] ego_data ego data with its trajectory and the rtree to populate
/// @param [in] params parameters
void make_ego_footprint_rtree(EgoData & ego_data, const PlannerParam & params);
}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop

#endif  // FOOTPRINT_HPP_
