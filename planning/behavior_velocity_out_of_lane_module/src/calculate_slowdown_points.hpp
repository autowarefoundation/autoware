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

#ifndef CALCULATE_SLOWDOWN_POINTS_HPP_
#define CALCULATE_SLOWDOWN_POINTS_HPP_

#include "footprint.hpp"
#include "types.hpp"

#include <motion_utils/trajectory/interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <boost/geometry/algorithms/overlaps.hpp>

#include <optional>
#include <vector>

namespace behavior_velocity_planner::out_of_lane
{

bool can_decelerate(
  const EgoData & ego_data, const PathPointWithLaneId & point, const double target_vel)
{
  if (ego_data.velocity < 0.5) return true;
  const auto dist_ahead_of_ego = motion_utils::calcSignedArcLength(
    ego_data.path.points, ego_data.pose.position, point.point.pose.position);
  const auto acc_to_target_vel =
    (ego_data.velocity * ego_data.velocity - target_vel * target_vel) / (2 * dist_ahead_of_ego);
  return acc_to_target_vel < std::abs(ego_data.max_decel);
}

std::optional<PathPointWithLaneId> calculate_last_in_lane_pose(
  const EgoData & ego_data, const Slowdown & decision, const Polygon2d & footprint,
  const PlannerParam & params)
{
  const auto from_arc_length =
    motion_utils::calcSignedArcLength(ego_data.path.points, 0, ego_data.first_path_idx);
  const auto to_arc_length =
    motion_utils::calcSignedArcLength(ego_data.path.points, 0, decision.target_path_idx);
  PathPointWithLaneId interpolated_point;
  for (auto l = to_arc_length - params.precision; l > from_arc_length; l -= params.precision) {
    // TODO(Maxime): binary search
    interpolated_point.point.pose = motion_utils::calcInterpolatedPose(ego_data.path.points, l);
    const auto respect_decel_limit =
      !params.skip_if_over_max_decel ||
      can_decelerate(ego_data, interpolated_point, decision.velocity);
    if (
      respect_decel_limit && !boost::geometry::overlaps(
                               project_to_pose(footprint, interpolated_point.point.pose),
                               decision.lane_to_avoid.polygon2d().basicPolygon()))
      return interpolated_point;
  }
  return std::nullopt;
}

/// @brief calculate the slowdown point to insert in the path
/// @param ego_data ego data (path, velocity, etc)
/// @param decisions decision (before which point to stop, what lane to avoid entering, etc)
/// @param params parameters
/// @return optional slowdown point to insert in the path
std::optional<SlowdownToInsert> calculate_slowdown_point(
  const EgoData & ego_data, const std::vector<Slowdown> & decisions, PlannerParam params)
{
  params.extra_front_offset += params.dist_buffer;
  const auto base_footprint = make_base_footprint(params);

  // search for the first slowdown decision for which a stop point can be inserted
  for (const auto & decision : decisions) {
    const auto last_in_lane_pose =
      calculate_last_in_lane_pose(ego_data, decision, base_footprint, params);
    if (last_in_lane_pose) return SlowdownToInsert{decision, *last_in_lane_pose};
  }
  return std::nullopt;
}
}  // namespace behavior_velocity_planner::out_of_lane
#endif  // CALCULATE_SLOWDOWN_POINTS_HPP_
