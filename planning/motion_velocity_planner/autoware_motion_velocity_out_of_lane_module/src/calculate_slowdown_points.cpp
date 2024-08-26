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
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{

std::optional<geometry_msgs::msg::Pose> calculate_last_avoiding_pose(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const universe_utils::Polygon2d & footprint, const lanelet::BasicPolygons2d & polygons_to_avoid,
  const double min_arc_length, const double max_arc_length, const double precision)
{
  geometry_msgs::msg::Pose interpolated_pose{};
  bool is_avoiding_pose = false;

  auto from = min_arc_length;
  auto to = max_arc_length;
  while (to - from > precision) {
    auto l = from + 0.5 * (to - from);
    interpolated_pose = motion_utils::calcInterpolatedPose(trajectory, l);
    const auto interpolated_footprint = project_to_pose(footprint, interpolated_pose);
    is_avoiding_pose =
      std::all_of(polygons_to_avoid.begin(), polygons_to_avoid.end(), [&](const auto & polygon) {
        return boost::geometry::disjoint(interpolated_footprint, polygon);
      });
    if (is_avoiding_pose) {
      from = l;
    } else {
      to = l;
    }
  }
  if (is_avoiding_pose) {
    return interpolated_pose;
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::Pose> calculate_pose_ahead_of_collision(
  const EgoData & ego_data, const OutOfLanePoint & point_to_avoid,
  const universe_utils::Polygon2d & footprint, const double precision)
{
  const auto first_avoid_arc_length = motion_utils::calcSignedArcLength(
    ego_data.trajectory_points, 0UL, point_to_avoid.trajectory_index);
  for (auto l = first_avoid_arc_length - precision; l >= ego_data.min_stop_arc_length;
       l -= precision) {
    const auto interpolated_pose =
      motion_utils::calcInterpolatedPose(ego_data.trajectory_points, l);
    const auto interpolated_footprint = project_to_pose(footprint, interpolated_pose);
    if (boost::geometry::intersects(interpolated_footprint, point_to_avoid.outside_ring)) {
      return interpolated_pose;
    }
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::Pose> calculate_slowdown_point(
  const EgoData & ego_data, const OutOfLaneData & out_of_lane_data, PlannerParam params)
{
  const auto point_to_avoid_it = std::find_if(
    out_of_lane_data.outside_points.cbegin(), out_of_lane_data.outside_points.cend(),
    [&](const auto & p) { return p.to_avoid; });
  if (point_to_avoid_it == out_of_lane_data.outside_points.cend()) {
    return std::nullopt;
  }
  const auto raw_footprint = make_base_footprint(params, true);  // ignore extra footprint offsets
  const auto base_footprint = make_base_footprint(params);
  params.extra_front_offset += params.lon_dist_buffer;
  params.extra_right_offset += params.lat_dist_buffer;
  params.extra_left_offset += params.lat_dist_buffer;
  const auto expanded_footprint = make_base_footprint(params);  // with added distance buffers
  lanelet::BasicPolygons2d polygons_to_avoid;
  for (const auto & ll : point_to_avoid_it->overlapped_lanelets) {
    polygons_to_avoid.push_back(ll.polygon2d().basicPolygon());
  }
  // points are ordered by trajectory index so the first one has the smallest index and arc length
  const auto first_outside_idx = out_of_lane_data.outside_points.front().trajectory_index;
  const auto first_outside_arc_length =
    motion_utils::calcSignedArcLength(ego_data.trajectory_points, 0UL, first_outside_idx);

  std::optional<geometry_msgs::msg::Pose> slowdown_point;
  // search for the first slowdown decision for which a stop point can be inserted
  // we first try to use the expanded footprint (distance buffers + extra footprint offsets)
  for (const auto & footprint : {expanded_footprint, base_footprint, raw_footprint}) {
    slowdown_point = calculate_last_avoiding_pose(
      ego_data.trajectory_points, footprint, polygons_to_avoid, ego_data.min_stop_arc_length,
      first_outside_arc_length, params.precision);
    if (slowdown_point) {
      break;
    }
  }
  // fallback to simply stopping ahead of the collision to avoid (regardless of being out of lane or
  // not)
  if (!slowdown_point) {
    slowdown_point = calculate_pose_ahead_of_collision(
      ego_data, *point_to_avoid_it, expanded_footprint, params.precision);
  }
  return slowdown_point;
}
}  // namespace autoware::motion_velocity_planner::out_of_lane
