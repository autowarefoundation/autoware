// Copyright 2024 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__KINEMATIC_BICYCLE_MODEL_HPP_
#define AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__KINEMATIC_BICYCLE_MODEL_HPP_

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <tf2/utils.h>

#include <vector>

namespace autoware::freespace_planning_algorithms
{
namespace kinematic_bicycle_model
{

static constexpr double eps = 0.001;

inline double getTurningRadius(const double base_length, const double steering_angle)
{
  return base_length / std::tan(steering_angle);
}

inline geometry_msgs::msg::Pose getPose(
  const geometry_msgs::msg::Pose & current_pose, const double base_length,
  const double steering_angle, const double distance)
{
  auto pose = current_pose;
  const double yaw = tf2::getYaw(current_pose.orientation);
  if (std::abs(steering_angle) < eps) {
    pose.position.x += distance * std::cos(yaw);
    pose.position.y += distance * std::sin(yaw);
    return pose;
  }

  const double R = getTurningRadius(base_length, steering_angle);
  const double beta = distance / R;
  pose.position.x += (R * std::sin(yaw + beta) - R * std::sin(yaw));
  pose.position.y += (R * std::cos(yaw) - R * std::cos(yaw + beta));
  pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw + beta);
  return pose;
}

}  // namespace kinematic_bicycle_model
}  // namespace autoware::freespace_planning_algorithms

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__KINEMATIC_BICYCLE_MODEL_HPP_
