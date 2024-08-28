// Copyright 2022-2024 TIER IV, Inc.
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

#include "obstacle_velocity_limiter.hpp"

#include "distance.hpp"
#include "forward_projection.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{

multi_polygon_t createPolygonMasks(
  const autoware_perception_msgs::msg::PredictedObjects & dynamic_obstacles, const double buffer,
  const double min_vel)
{
  return createObjectPolygons(dynamic_obstacles, buffer, min_vel);
}

std::vector<polygon_t> createFootprintPolygons(
  const std::vector<multi_linestring_t> & projected_linestrings, const double lateral_offset)
{
  std::vector<polygon_t> footprints;
  footprints.reserve(projected_linestrings.size());
  for (const auto & linestrings : projected_linestrings) {
    footprints.push_back(generateFootprint(linestrings, lateral_offset));
  }
  return footprints;
}

polygon_t createTrajectoryFootprint(
  const TrajectoryPoints & trajectory, const double lateral_offset)
{
  linestring_t ls;
  ls.reserve(trajectory.size());
  for (const auto & p : trajectory) ls.emplace_back(p.pose.position.x, p.pose.position.y);
  return generateFootprint(ls, lateral_offset);
}

polygon_t createEnvelopePolygon(
  const TrajectoryPoints & trajectory, const size_t start_idx,
  ProjectionParameters & projection_params)
{
  polygon_t envelope_polygon;
  const auto trajectory_size = trajectory.size() - start_idx;
  if (trajectory_size < 2) return envelope_polygon;

  envelope_polygon.outer().resize(trajectory_size * 2 + 1);
  for (size_t i = 0; i < trajectory_size; ++i) {
    const auto & point = trajectory[i + start_idx];
    projection_params.update(point);
    const auto forward_simulated_vector =
      forwardSimulatedSegment(point.pose.position, projection_params);
    envelope_polygon.outer()[i] = forward_simulated_vector.second;
    const auto reverse_index = 2 * trajectory_size - i - 1;
    envelope_polygon.outer()[reverse_index] = forward_simulated_vector.first;
  }
  envelope_polygon.outer().push_back(envelope_polygon.outer().front());
  boost::geometry::correct(envelope_polygon);
  return envelope_polygon;
}

polygon_t createEnvelopePolygon(const std::vector<polygon_t> & footprints)
{
  multi_polygon_t unions;
  multi_polygon_t result;
  for (const auto & footprint : footprints) {
    boost::geometry::union_(footprint, unions, result);
    unions = result;
    boost::geometry::clear(result);
  }
  if (unions.empty()) return {};
  return unions.front();
}

std::vector<multi_linestring_t> createProjectedLines(
  const TrajectoryPoints & trajectory, ProjectionParameters & params)
{
  std::vector<multi_linestring_t> projections;
  projections.reserve(trajectory.size());
  for (const auto & point : trajectory) {
    params.update(point);
    if (params.model == ProjectionParameters::PARTICLE) {
      const auto projection = forwardSimulatedSegment(point.pose.position, params);
      projections.push_back({{projection.first, projection.second}});
    } else {
      projections.push_back(bicycleProjectionLines(point.pose.position, params));
    }
  }
  return projections;
}

std::vector<autoware::motion_velocity_planner::SlowdownInterval> calculate_slowdown_intervals(
  TrajectoryPoints & trajectory, const CollisionChecker & collision_checker,
  const std::vector<multi_linestring_t> & projections, const std::vector<polygon_t> & footprints,
  ProjectionParameters & projection_params, const VelocityParameters & velocity_params,
  autoware::motion_utils::VirtualWalls & virtual_walls)
{
  std::vector<autoware::motion_velocity_planner::SlowdownInterval> slowdown_intervals;
  size_t previous_slowdown_index = trajectory.size();
  for (size_t i = 0; i < trajectory.size(); ++i) {
    auto & trajectory_point = trajectory[i];
    // First linestring is used to calculate distance
    if (projections[i].empty()) continue;
    projection_params.update(trajectory_point);
    const auto dist_to_collision = distanceToClosestCollision(
      projections[i][0], footprints[i], collision_checker, projection_params);
    if (dist_to_collision) {
      const auto min_feasible_velocity =
        velocity_params.current_ego_velocity -
        velocity_params.max_deceleration *
          rclcpp::Duration(trajectory_point.time_from_start).seconds();

      const auto safe_velocity = std::max(
        static_cast<double>(*dist_to_collision - projection_params.extra_length) /
          static_cast<double>(projection_params.duration),
        velocity_params.min_velocity);
      slowdown_intervals.emplace_back(
        trajectory_point.pose.position, trajectory_point.pose.position,
        std::max(min_feasible_velocity, safe_velocity));

      // with consecutive slowdowns only add a virtual wall for the first one
      if (previous_slowdown_index + 1 != i) {
        autoware::motion_utils::VirtualWall wall;
        wall.pose = trajectory_point.pose;
        virtual_walls.push_back(wall);
      }
      previous_slowdown_index = i;
    }
  }
  return slowdown_intervals;
}
}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
