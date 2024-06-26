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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__TRAJECTORY_PREPROCESSING_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__TRAJECTORY_PREPROCESSING_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::motion_velocity_planner
{
/// @brief downsample a trajectory, reducing its number of points by the given factor
/// @param[in] trajectory input trajectory
/// @param[in] start_idx starting index of the input trajectory
/// @param[in] end_idx ending index of the input trajectory
/// @param[in] factor factor used for downsampling
/// @return downsampled trajectory
std::vector<autoware_planning_msgs::msg::TrajectoryPoint> downsample_trajectory(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const size_t start_idx, const size_t end_idx, const int factor)
{
  if (factor < 1) {
    return trajectory;
  }
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> downsampled_traj;
  downsampled_traj.reserve((end_idx - start_idx) / factor + 1);
  for (size_t i = start_idx; i <= end_idx; i += factor) {
    downsampled_traj.push_back(trajectory[i]);
  }
  return downsampled_traj;
}
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__TRAJECTORY_PREPROCESSING_HPP_
