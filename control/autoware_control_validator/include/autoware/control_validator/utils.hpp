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

#ifndef AUTOWARE__CONTROL_VALIDATOR__UTILS_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>

namespace autoware::control_validator
{
/**
 * @brief Shift pose along the yaw direction
 */
void shift_pose(geometry_msgs::msg::Pose & pose, double longitudinal);

/**
 * @brief Calculate the maximum lateral distance between the reference trajectory and the predicted
 * trajectory
 * @param reference_trajectory reference trajectory
 * @param predicted_trajectory predicted trajectory
 */
double calc_max_lateral_distance(
  const autoware_planning_msgs::msg::Trajectory & reference_trajectory,
  const autoware_planning_msgs::msg::Trajectory & predicted_trajectory);
}  // namespace autoware::control_validator

#endif  // AUTOWARE__CONTROL_VALIDATOR__UTILS_HPP_
