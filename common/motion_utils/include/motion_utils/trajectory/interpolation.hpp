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
#include "tier4_autoware_utils/math/constants.hpp"

#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

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

}  // namespace motion_utils

#endif  // MOTION_UTILS__TRAJECTORY__INTERPOLATION_HPP_
