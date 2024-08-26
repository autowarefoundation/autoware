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

#ifndef CALCULATE_SLOWDOWN_POINTS_HPP_
#define CALCULATE_SLOWDOWN_POINTS_HPP_

#include "types.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <optional>

namespace autoware::motion_velocity_planner::out_of_lane
{
/// @brief calculate the last pose along the trajectory where ego does not go out of lane
/// @param [in] ego_data ego data
/// @param [in] footprint the ego footprint
/// @param [in] min_arc_length minimum arc length for the search
/// @param [in] max_arc_length maximum arc length for the search
/// @param [in] precision [m] search precision
/// @return the last pose that is not out of lane (if found)
std::optional<geometry_msgs::msg::Pose> calculate_last_in_lane_pose(
  const EgoData & ego_data, const autoware::universe_utils::Polygon2d & footprint,
  const double min_arc_length, const double max_arc_length, const double precision);

/// @brief calculate the slowdown pose just ahead of a point to avoid
/// @param ego_data ego data (trajectory, velocity, etc)
/// @param point_to_avoid the point to avoid
/// @param footprint the ego footprint
/// @param params parameters
/// @return optional slowdown point to insert in the trajectory
std::optional<geometry_msgs::msg::Pose> calculate_pose_ahead_of_collision(
  const EgoData & ego_data, const OutOfLanePoint & point_to_avoid,
  const universe_utils::Polygon2d & footprint, const double precision);

/// @brief calculate the slowdown point to insert in the trajectory
/// @param ego_data ego data (trajectory, velocity, etc)
/// @param out_of_lane_data data about out of lane areas
/// @param params parameters
/// @return optional slowdown point to insert in the trajectory
std::optional<geometry_msgs::msg::Pose> calculate_slowdown_point(
  const EgoData & ego_data, const OutOfLaneData & out_of_lane_data, PlannerParam params);
}  // namespace autoware::motion_velocity_planner::out_of_lane
#endif  // CALCULATE_SLOWDOWN_POINTS_HPP_
