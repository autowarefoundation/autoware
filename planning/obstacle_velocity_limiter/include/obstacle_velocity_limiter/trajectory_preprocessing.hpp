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

#ifndef OBSTACLE_VELOCITY_LIMITER__TRAJECTORY_PREPROCESSING_HPP_
#define OBSTACLE_VELOCITY_LIMITER__TRAJECTORY_PREPROCESSING_HPP_

#include "obstacle_velocity_limiter/types.hpp"

namespace obstacle_velocity_limiter
{
/// @brief calculate trajectory index that is ahead of the given index by the given distance
/// @param[in] trajectory trajectory
/// @param[in] ego_idx index closest to the current ego position in the trajectory
/// @param[in] start_distance desired distance ahead of the ego_idx
/// @return trajectory index ahead of ego_idx by the start_distance
size_t calculateStartIndex(
  const Trajectory & trajectory, const size_t ego_idx, const Float start_distance);

/// @brief calculate trajectory index that is ahead of the given index by the given distance
/// @param[in] trajectory trajectory
/// @param[in] start_idx starting index
/// @param[in] max_length maximum length from start_idx to the returned index
/// @param[in] max_duration maximum duration from start_idx to the returned index
/// @return trajectory index ahead of the start_idx by at most the given length and duration
size_t calculateEndIndex(
  const Trajectory & trajectory, const size_t start_idx, const Float max_length,
  const Float max_duration);

/// @brief downsample a trajectory, reducing its number of points by the given factor
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index of the input trajectory
/// @param[in] end_idx ending index of the input trajectory
/// @param[in] factor factor used for downsampling
/// @return downsampled trajectory
Trajectory downsampleTrajectory(
  const Trajectory & trajectory, const size_t start_idx, const size_t end_idx, const int factor);

/// @brief recalculate the steering angle of the trajectory
/// @details uses the change in headings for calculation
/// @param[inout] trajectory input trajectory
/// @param[in] wheel_base wheel base of the vehicle
void calculateSteeringAngles(Trajectory & trajectory, const Float wheel_base);

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

#endif  // OBSTACLE_VELOCITY_LIMITER__TRAJECTORY_PREPROCESSING_HPP_
