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

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <vector>

namespace behavior_velocity_planner
{
namespace out_of_lane
{
/// @brief create the base footprint of ego
/// @param [in] p parameters used to create the footprint
/// @param [in] ignore_offset optional parameter, if true, ignore the "extra offsets" to build the
/// footprint
/// @return base ego footprint
tier4_autoware_utils::Polygon2d make_base_footprint(
  const PlannerParam & p, const bool ignore_offset = false);
/// @brief project a footprint to the given pose
/// @param [in] base_footprint footprint to project
/// @param [in] pose projection pose
/// @return footprint projected to the given pose
lanelet::BasicPolygon2d project_to_pose(
  const tier4_autoware_utils::Polygon2d & base_footprint, const geometry_msgs::msg::Pose & pose);
/// @brief calculate the path footprints
/// @details the resulting polygon follows the format used by the lanelet library: clockwise order
/// and implicit closing edge
/// @param [in] ego_data data related to the ego vehicle (includes its path)
/// @param [in] params parameters
/// @return polygon footprints for each path point starting from ego's current position
std::vector<lanelet::BasicPolygon2d> calculate_path_footprints(
  const EgoData & ego_data, const PlannerParam & params);
/// @brief calculate the current ego footprint
/// @param [in] ego_data data related to the ego vehicle
/// @param [in] params parameters
/// @param [in] ignore_offset optional parameter, if true, ignore the "extra offsets" to build the
/// footprint
lanelet::BasicPolygon2d calculate_current_ego_footprint(
  const EgoData & ego_data, const PlannerParam & params, const bool ignore_offset = false);
}  // namespace out_of_lane
}  // namespace behavior_velocity_planner

#endif  // FOOTPRINT_HPP_
