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

#ifndef TRAJECTORY_PREPROCESSING_HPP_
#define TRAJECTORY_PREPROCESSING_HPP_

#include "types.hpp"

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{
/// @brief calculate trajectory index that is ahead of the given index by the given distance
/// @param[in] trajectory trajectory
/// @param[in] ego_idx index closest to the current ego position in the trajectory
/// @param[in] start_distance desired distance ahead of the ego_idx
/// @return trajectory index ahead of ego_idx by the start_distance
size_t calculateStartIndex(
  const TrajectoryPoints & trajectory, const size_t ego_idx, const double start_distance);

/// @brief calculate trajectory index that is ahead of the given index by the given distance
/// @param[in] trajectory trajectory
/// @param[in] start_idx starting index
/// @param[in] max_length maximum length from start_idx to the returned index
/// @param[in] max_duration maximum duration from start_idx to the returned index
/// @return trajectory index ahead of the start_idx by at most the given length and duration
size_t calculateEndIndex(
  const TrajectoryPoints & trajectory, const size_t start_idx, const double max_length,
  const double max_duration);

/// @brief downsample a trajectory, reducing its number of points by the given factor
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index of the input trajectory
/// @param[in] end_idx ending index of the input trajectory
/// @param[in] factor factor used for downsampling
/// @return downsampled trajectory
TrajectoryPoints downsampleTrajectory(
  const TrajectoryPoints & trajectory, const size_t start_idx, const size_t end_idx,
  const int64_t factor);

/// @brief recalculate the steering angle of the trajectory
/// @details uses the change in headings for calculation
/// @param[inout] trajectory input trajectory
/// @param[in] wheel_base wheel base of the vehicle
void calculateSteeringAngles(TrajectoryPoints & trajectory, const double wheel_base);

/// @brief insert a point in the trajectory and fill its velocity and acceleration data
/// @param[inout] trajectory input trajectory
/// @param[in] point point to insert in the trajectory
void add_trajectory_point(TrajectoryPoints & trajectory, const geometry_msgs::msg::Point & point);
}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter

#endif  // TRAJECTORY_PREPROCESSING_HPP_
