// Copyright 2020 Tier IV, Inc.
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

#ifndef TIER4_PLANNING_RVIZ_PLUGIN__UTILS_HPP_
#define TIER4_PLANNING_RVIZ_PLUGIN__UTILS_HPP_

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>

#include <tf2/utils.h>

namespace rviz_plugins
{
template <class T>
bool isDrivingForward(const T points_with_twist, size_t target_idx)
{
  constexpr double epsilon = 1e-6;

  // 1. check velocity
  const double target_velocity =
    tier4_autoware_utils::getLongitudinalVelocity(points_with_twist.at(target_idx));
  if (epsilon < target_velocity) {
    return true;
  } else if (target_velocity < -epsilon) {
    return false;
  }

  // 2. check points size is enough
  if (points_with_twist.size() < 2) {
    return true;
  }

  // 3. check points direction
  const bool is_last_point = target_idx == points_with_twist.size() - 1;
  const size_t first_idx = is_last_point ? target_idx - 1 : target_idx;
  const size_t second_idx = is_last_point ? target_idx : target_idx + 1;

  const auto first_pose = tier4_autoware_utils::getPose(points_with_twist.at(first_idx));
  const auto second_pose = tier4_autoware_utils::getPose(points_with_twist.at(second_idx));
  const double first_traj_yaw = tf2::getYaw(first_pose.orientation);
  const double driving_direction_yaw =
    tier4_autoware_utils::calcAzimuthAngle(first_pose.position, second_pose.position);
  if (
    std::abs(tier4_autoware_utils::normalizeRadian(first_traj_yaw - driving_direction_yaw)) <
    M_PI_2) {
    return true;
  }

  return false;
}
}  // namespace rviz_plugins

#endif  // TIER4_PLANNING_RVIZ_PLUGIN__UTILS_HPP_
