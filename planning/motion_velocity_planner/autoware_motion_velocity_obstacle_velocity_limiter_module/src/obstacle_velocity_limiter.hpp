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

#ifndef OBSTACLE_VELOCITY_LIMITER_HPP_
#define OBSTACLE_VELOCITY_LIMITER_HPP_

#include "obstacles.hpp"
#include "parameters.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{
/// @brief calculate trajectory index that is ahead of the given index by the given distance
/// @param[in] trajectory trajectory
/// @param[in] ego_idx index closest to the current ego position in the trajectory
/// @param[in] start_distance desired distance ahead of the ego_idx
/// @return trajectory index ahead of ego_idx by the start_distance
size_t calculateStartIndex(
  const TrajectoryPoints & trajectory, const size_t ego_idx, const double start_distance);

/// @brief downsample a trajectory, reducing its number of points by the given factor
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index of the input trajectory
/// @param[in] factor factor used for downsampling
/// @return downsampled trajectory
TrajectoryPoints downsampleTrajectory(
  const TrajectoryPoints & trajectory, const size_t start_idx, const int factor);

/// @brief recalculate the steering angle of the trajectory
/// @details uses the change in headings for calculation
/// @param[inout] trajectory input trajectory
/// @param[in] wheel_base wheel base of the vehicle
void calculateSteeringAngles(TrajectoryPoints & trajectory, const double wheel_base);

/// @brief create negative polygon masks from the dynamic objects
/// @param[in] dynamic_obstacles the dynamic objects to mask
/// @param[in] buffer buffer used to enlarge the mask
/// @param[in] min_vel minimum velocity for an object to be masked
/// @return polygon masks around dynamic objects
multi_polygon_t createPolygonMasks(
  const autoware_perception_msgs::msg::PredictedObjects & dynamic_obstacles, const double buffer,
  const double min_vel);

/// @brief create footprint polygons from projection lines
/// @details A footprint is create for each group of lines. Each group of lines is assumed to share
/// some points such that their footprint is a single connected polygon.
/// @param[in] projections the projection lines
/// @param[in] lateral_offset offset to create polygons around the lines
/// @return polygon footprint of each projection lines
std::vector<polygon_t> createFootprintPolygons(
  const std::vector<multi_linestring_t> & projected_linestrings, const double lateral_offset);

/// @brief create the footprint polygon from a trajectory
/// @param[in] trajectory the trajectory for which to create a footprint
/// @param[in] lateral_offset offset to create polygons around the trajectory points
/// @return polygon footprint of the trajectory
polygon_t createTrajectoryFootprint(
  const TrajectoryPoints & trajectory, const double lateral_offset);

/// @brief create a polygon of the safety envelope
/// @details the safety envelope is the area covered by forward projections at each trajectory
/// point
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index in the input trajectory
/// @param[in] projection_params parameters of the forward projection
/// @return the envelope polygon
polygon_t createEnvelopePolygon(
  const TrajectoryPoints & trajectory, const size_t start_idx,
  ProjectionParameters & projection_params);

/// @brief create a polygon of the safety envelope from the projection footprints
/// @details the safety envelope is the area covered by forward projections at each trajectory
/// point
/// @param[in] footprints projection footprints
/// @return the envelope polygon
polygon_t createEnvelopePolygon(const std::vector<polygon_t> & footprints);

/// @brief create projection lines for each trajectory point
/// @details depending on the method used, multiple lines can be created for a same trajectory point
/// @param[in] trajectory input trajectory
/// @param[in] params projection parameters
/// @return projection lines for each trajectory point
std::vector<multi_linestring_t> createProjectedLines(
  const TrajectoryPoints & trajectory, ProjectionParameters & params);

/// @brief calculate slowdown intervals on the given trajectory
/// @param[in] trajectory input trajectory
/// @param[in] collision_checker object used to retrieve collision points
/// @param[in] projections forward projection lines at each trajectory point
/// @param[in] footprints footprint of the forward projection at each trajectory point
/// @param[in] projection_params projection parameters
/// @param[in] velocity_params velocity parameters
/// @return slowdown intervals
std::vector<autoware::motion_velocity_planner::SlowdownInterval> calculate_slowdown_intervals(
  TrajectoryPoints & trajectory, const CollisionChecker & collision_checker,
  const std::vector<multi_linestring_t> & projections, const std::vector<polygon_t> & footprints,
  ProjectionParameters & projection_params, const VelocityParameters & velocity_params,
  autoware::motion_utils::VirtualWalls & virtual_walls);

}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter

#endif  // OBSTACLE_VELOCITY_LIMITER_HPP_
