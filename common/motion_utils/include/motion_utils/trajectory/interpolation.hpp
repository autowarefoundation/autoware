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

#ifndef MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_
#define MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include "autoware_auto_planning_msgs/msg/detail/path_with_lane_id__struct.hpp"
#include "autoware_auto_planning_msgs/msg/detail/trajectory__struct.hpp"

#include <algorithm>
#include <limits>

namespace motion_utils
{
/**
 * @brief An interpolation function that finds the closest interpolated point on the trajectory from
 * the given pose
 * @param trajectory input trajectory
 * @param target_pose target_pose
 * @param use_zero_order_for_twist flag to decide wether to use zero order hold interpolation for
 * twist information
 * @return resampled path(poses)
 */
autoware_auto_planning_msgs::msg::TrajectoryPoint calcInterpolatedPoint(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & target_pose, const bool use_zero_order_hold_for_twist = false,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());

/**
 * @brief An interpolation function that finds the closest interpolated point on the path from
 * the given pose
 * @param path input path
 * @param target_pose target_pose
 * @param use_zero_order_for_twist flag to decide wether to use zero order hold interpolation for
 * twist information
 * @return resampled path(poses)
 */
autoware_auto_planning_msgs::msg::PathPointWithLaneId calcInterpolatedPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & target_pose, const bool use_zero_order_hold_for_twist = false,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max());

/**
 * @brief An interpolation function that finds the closest interpolated point on the path that is a
 * certain length away from the given pose
 * @param points input path
 * @param target_length length from the front point of the path
 * @return resampled pose
 */
template <class T>
geometry_msgs::msg::Pose calcInterpolatedPose(const T & points, const double target_length)
{
  if (points.empty()) {
    geometry_msgs::msg::Pose interpolated_pose;
    return interpolated_pose;
  }

  if (points.size() < 2 || target_length < 0.0) {
    return tier4_autoware_utils::getPose(points.front());
  }

  double accumulated_length = 0;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    const auto & curr_pose = tier4_autoware_utils::getPose(points.at(i));
    const auto & next_pose = tier4_autoware_utils::getPose(points.at(i + 1));
    const double length = tier4_autoware_utils::calcDistance3d(curr_pose, next_pose);
    if (accumulated_length + length > target_length) {
      const double ratio = (target_length - accumulated_length) / std::max(length, 1e-6);
      return tier4_autoware_utils::calcInterpolatedPose(curr_pose, next_pose, ratio);
    }
    accumulated_length += length;
  }

  return tier4_autoware_utils::getPose(points.back());
}

}  // namespace motion_utils

#endif  // MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_
