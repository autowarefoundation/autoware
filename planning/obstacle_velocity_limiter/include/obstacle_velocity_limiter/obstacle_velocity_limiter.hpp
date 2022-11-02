// Copyright 2022 TIER IV, Inc.
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

#ifndef OBSTACLE_VELOCITY_LIMITER__OBSTACLE_VELOCITY_LIMITER_HPP_
#define OBSTACLE_VELOCITY_LIMITER__OBSTACLE_VELOCITY_LIMITER_HPP_

#include "obstacle_velocity_limiter/obstacles.hpp"
#include "obstacle_velocity_limiter/parameters.hpp"
#include "obstacle_velocity_limiter/types.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <string>
#include <vector>

namespace obstacle_velocity_limiter
{

/// @brief calculate the apparent safe velocity
/// @param[in] trajectory_point trajectory point for which to calculate the apparent safe velocity
/// @param[in] dist_to_collision distance from the trajectory point to the apparent collision
/// @return apparent safe velocity
Float calculateSafeVelocity(
  const TrajectoryPoint & trajectory_point, const Float dist_to_collision);

/// @brief calculate trajectory index that is ahead of the given index by the given distance
/// @param[in] trajectory trajectory
/// @param[in] ego_idx index closest to the current ego position in the trajectory
/// @param[in] start_distance desired distance ahead of the ego_idx
/// @return trajectory index ahead of ego_idx by the start_distance
size_t calculateStartIndex(
  const Trajectory & trajectory, const size_t ego_idx, const Float start_distance);

/// @brief downsample a trajectory, reducing its number of points by the given factor
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index of the input trajectory
/// @param[in] factor factor used for downsampling
/// @return downsampled trajectory
Trajectory downsampleTrajectory(
  const Trajectory & trajectory, const size_t start_idx, const int factor);

/// @brief recalculate the steering angle of the trajectory
/// @details uses the change in headings for calculation
/// @param[inout] trajectory input trajectory
/// @param[in] wheel_base wheel base of the vehicle
void calculateSteeringAngles(Trajectory & trajectory, const Float wheel_base);

/// @brief create negative polygon masks from the dynamic objects
/// @param[in] dynamic_obstacles the dynamic objects to mask
/// @param[in] buffer buffer used to enlarge the mask
/// @param[in] min_vel minimum velocity for an object to be masked
/// @return polygon masks around dynamic objects
multipolygon_t createPolygonMasks(
  const autoware_auto_perception_msgs::msg::PredictedObjects & dynamic_obstacles,
  const Float buffer, const Float min_vel);

/// @brief create footprint polygons from projection lines
/// @details A footprint is create for each group of lines. Each group of lines is assumed to share
/// some points such that their footprint is a single connected polygon.
/// @param[in] projections the projection lines
/// @param[in] lateral_offset offset to create polygons around the lines
/// @return polygon footprint of each projection lines
std::vector<polygon_t> createFootprintPolygons(
  const std::vector<multilinestring_t> & projected_linestrings, const Float lateral_offset);

/// @brief create the footprint polygon from a trajectory
/// @param[in] trajectory the trajectory for which to create a footprint
/// @param[in] lateral_offset offset to create polygons around the trajectory points
/// @return polygon footprint of the trajectory
polygon_t createTrajectoryFootprint(const Trajectory & trajectory, const Float lateral_offset);

/// @brief create a polygon of the safety envelope
/// @details the safety envelope is the area covered by forward projections at each trajectory
/// point
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index in the input trajectory
/// @param[in] projection_params parameters of the forward projection
/// @return the envelope polygon
polygon_t createEnvelopePolygon(
  const Trajectory & trajectory, const size_t start_idx, ProjectionParameters & projection_params);

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
/// @return projecton lines for each trajectory point
std::vector<multilinestring_t> createProjectedLines(
  const Trajectory & trajectory, ProjectionParameters & params);

/// @brief limit the velocity of the given trajectory
/// @param[in] trajectory input trajectory
/// @param[in] collision_checker object used to retrive collision points
/// @param[in] projections forward projection lines at each trajectory point
/// @param[in] footprints footprint of the forward projection at each trajectory point
/// @param[in] projection_params projection parameters
/// @param[in] velocity_params velocity parameters
void limitVelocity(
  Trajectory & trajectory, const CollisionChecker & collision_checker,
  const std::vector<multilinestring_t> & projections, const std::vector<polygon_t> & footprints,
  ProjectionParameters & projection_params, const VelocityParameters & velocity_params);

/// @brief copy the velocity profile of a downsampled trajectory to the original trajectory
/// @param[in] downsampled_trajectory downsampled trajectory
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index of the downsampled trajectory relative to the input
/// @param[in] factor downsampling factor
/// @return input trajectory with the velocity profile of the downsampled trajectory
Trajectory copyDownsampledVelocity(
  const Trajectory & downsampled_traj, Trajectory trajectory, const size_t start_idx,
  const int factor);
}  // namespace obstacle_velocity_limiter

#endif  // OBSTACLE_VELOCITY_LIMITER__OBSTACLE_VELOCITY_LIMITER_HPP_
