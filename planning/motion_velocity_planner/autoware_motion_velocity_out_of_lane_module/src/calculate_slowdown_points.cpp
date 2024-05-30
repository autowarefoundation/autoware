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

#include "calculate_slowdown_points.hpp"

#include "footprint.hpp"

#include <motion_utils/trajectory/interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <boost/geometry/algorithms/overlaps.hpp>

#include <lanelet2_core/geometry/Polygon.h>

namespace autoware::motion_velocity_planner::out_of_lane
{

bool can_decelerate(
  const EgoData & ego_data, const TrajectoryPoint & point, const double target_vel)
{
  // TODO(Maxime): use the library function
  const auto dist_ahead_of_ego = motion_utils::calcSignedArcLength(
    ego_data.trajectory_points, ego_data.pose.position, point.pose.position);
  const auto acc_to_target_vel =
    (ego_data.velocity * ego_data.velocity - target_vel * target_vel) / (2 * dist_ahead_of_ego);
  return acc_to_target_vel < std::abs(ego_data.max_decel);
}

std::optional<TrajectoryPoint> calculate_last_in_lane_pose(
  const EgoData & ego_data, const Slowdown & decision,
  const tier4_autoware_utils::Polygon2d & footprint,
  const std::optional<SlowdownToInsert> & prev_slowdown_point, const PlannerParam & params)
{
  const auto from_arc_length =
    motion_utils::calcSignedArcLength(ego_data.trajectory_points, 0, ego_data.first_trajectory_idx);
  const auto to_arc_length = motion_utils::calcSignedArcLength(
    ego_data.trajectory_points, 0, decision.target_trajectory_idx);
  TrajectoryPoint interpolated_point;
  for (auto l = to_arc_length - params.precision; l > from_arc_length; l -= params.precision) {
    // TODO(Maxime): binary search
    interpolated_point.pose = motion_utils::calcInterpolatedPose(ego_data.trajectory_points, l);
    const auto respect_decel_limit =
      !params.skip_if_over_max_decel || prev_slowdown_point ||
      can_decelerate(ego_data, interpolated_point, decision.velocity);
    const auto interpolated_footprint = project_to_pose(footprint, interpolated_point.pose);
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
}  // namespace autoware::motion_velocity_planner::out_of_lane
