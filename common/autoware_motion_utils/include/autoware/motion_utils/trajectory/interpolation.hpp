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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_

#include "autoware/universe_utils/geometry/geometry.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "tier4_planning_msgs/msg/path_with_lane_id.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>

namespace autoware::motion_utils
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
autoware_planning_msgs::msg::TrajectoryPoint calcInterpolatedPoint(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
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
tier4_planning_msgs::msg::PathPointWithLaneId calcInterpolatedPoint(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
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
    return autoware::universe_utils::getPose(points.front());
  }

  double accumulated_length = 0;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    const auto & curr_pose = autoware::universe_utils::getPose(points.at(i));
    const auto & next_pose = autoware::universe_utils::getPose(points.at(i + 1));
    const double length = autoware::universe_utils::calcDistance3d(curr_pose, next_pose);
    if (accumulated_length + length > target_length) {
      const double ratio = (target_length - accumulated_length) / std::max(length, 1e-6);
      return autoware::universe_utils::calcInterpolatedPose(curr_pose, next_pose, ratio);
    }
    accumulated_length += length;
  }

  return autoware::universe_utils::getPose(points.back());
}

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_
