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

#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{

/// @brief estimate whether ego can decelerate without breaking the deceleration limit
/// @details assume ego wants to reach the target point at the target velocity
/// @param [in] ego_data ego data
/// @param [in] point target point
/// @param [in] target_vel target_velocity
bool can_decelerate(
  const EgoData & ego_data, const TrajectoryPoint & point, const double target_vel);

/// @brief calculate the last pose along the trajectory where ego does not overlap the lane to avoid
/// @param [in] ego_data ego data
/// @param [in] decision the input decision (i.e., which lane to avoid and at what speed)
/// @param [in] footprint the ego footprint
/// @param [in] prev_slowdown_point previous slowdown point. If set, ignore deceleration limits
/// @param [in] params parameters
/// @return the last pose that is not out of lane (if found)
std::optional<TrajectoryPoint> calculate_last_in_lane_pose(
  const EgoData & ego_data, const Slowdown & decision,
  const autoware::universe_utils::Polygon2d & footprint,
  const std::optional<SlowdownToInsert> & prev_slowdown_point, const PlannerParam & params);

/// @brief calculate the slowdown point to insert in the trajectory
/// @param ego_data ego data (trajectory, velocity, etc)
/// @param decisions decision (before which point to stop, what lane to avoid entering, etc)
/// @param prev_slowdown_point previously calculated slowdown point
/// @param params parameters
/// @return optional slowdown point to insert in the trajectory
std::optional<SlowdownToInsert> calculate_slowdown_point(
  const EgoData & ego_data, const std::vector<Slowdown> & decisions,
  const std::optional<SlowdownToInsert> prev_slowdown_point, PlannerParam params);
}  // namespace autoware::motion_velocity_planner::out_of_lane
#endif  // CALCULATE_SLOWDOWN_POINTS_HPP_
