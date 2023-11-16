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

/// @brief estimate whether ego can decelerate without breaking the deceleration limit
/// @details assume ego wants to reach the target point at the target velocity
/// @param [in] ego_data ego data
/// @param [in] point target point
/// @param [in] target_vel target_velocity
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

/// @brief calculate the last pose along the path where ego does not overlap the lane to avoid
/// @param [in] ego_data ego data
/// @param [in] decision the input decision (i.e., which lane to avoid and at what speed)
/// @param [in] footprint the ego footprint
/// @param [in] prev_slowdown_point previous slowdown point. If set, ignore deceleration limits
/// @param [in] params parameters
/// @return the last pose that is not out of lane (if found)
std::optional<PathPointWithLaneId> calculate_last_in_lane_pose(
  const EgoData & ego_data, const Slowdown & decision, const Polygon2d & footprint,
  const std::optional<SlowdownToInsert> & prev_slowdown_point, const PlannerParam & params)
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
      !params.skip_if_over_max_decel || prev_slowdown_point ||
      can_decelerate(ego_data, interpolated_point, decision.velocity);
    const auto interpolated_footprint = project_to_pose(footprint, interpolated_point.point.pose);
    const auto is_overlap_lane = boost::geometry::overlaps(
      interpolated_footprint, decision.lane_to_avoid.polygon2d().basicPolygon());
    const auto is_overlap_extra_lane =
      prev_slowdown_point &&
      boost::geometry::overlaps(
        interpolated_footprint,
        prev_slowdown_point->slowdown.lane_to_avoid.polygon2d().basicPolygon());
    if (respect_decel_limit && !is_overlap_lane && !is_overlap_extra_lane)
      return interpolated_point;
  }
  return std::nullopt;
}

/// @brief calculate the slowdown point to insert in the path
/// @param ego_data ego data (path, velocity, etc)
/// @param decisions decision (before which point to stop, what lane to avoid entering, etc)
/// @param prev_slowdown_point previously calculated slowdown point
/// @param params parameters
/// @return optional slowdown point to insert in the path
std::optional<SlowdownToInsert> calculate_slowdown_point(
  const EgoData & ego_data, const std::vector<Slowdown> & decisions,
  const std::optional<SlowdownToInsert> prev_slowdown_point, PlannerParam params)
{
  params.extra_front_offset += params.dist_buffer;
  const auto base_footprint = make_base_footprint(params);

  // search for the first slowdown decision for which a stop point can be inserted
  for (const auto & decision : decisions) {
    const auto last_in_lane_pose =
      calculate_last_in_lane_pose(ego_data, decision, base_footprint, prev_slowdown_point, params);
    if (last_in_lane_pose) return SlowdownToInsert{decision, *last_in_lane_pose};
  }
  return std::nullopt;
}
}  // namespace behavior_velocity_planner::out_of_lane
#endif  // CALCULATE_SLOWDOWN_POINTS_HPP_
