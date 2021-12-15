// Copyright 2021 the Autoware Foundation
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

#ifndef MOTION_COMMON__TRAJECTORY_COMMON_HPP_
#define MOTION_COMMON__TRAJECTORY_COMMON_HPP_

#include <experimental/optional>
#include <limits>
#include <stdexcept>
#include <vector>

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "common/types.hpp"
#include "eigen3/Eigen/Core"
#include "geometry/common_2d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "helper_functions/angle_utils.hpp"
#include "motion_common/motion_common.hpp"
#include "tf2/utils.h"

namespace autoware
{
namespace motion
{
namespace motion_common
{
typedef autoware_auto_planning_msgs::msg::TrajectoryPoint Point;
typedef decltype (autoware_auto_planning_msgs::msg::Trajectory::points) Points;
using autoware::common::types::float64_t;
typedef Eigen::Matrix<float64_t, 3, 1> Vector3f;

/**
 * @brief throws an exception if the given list of points is empty
 * @param [in] points list of points to check
 */
MOTION_COMMON_PUBLIC void validateNonEmpty(const Points & points);

/**
 * @brief calculate the yaw deviation between two angles
 * @param [in] base_yaw base yaw angle [radians]
 * @param [in] target_yaw target yaw angle [radians]
 * @return normalized angle from the base to the target [radians]
 */
MOTION_COMMON_PUBLIC float64_t calcYawDeviation(
  const float64_t & base_yaw,
  const float64_t & target_yaw);

/**
 * @brief search first index with a velocity of zero in the given range of points
 * @param [in] points list of points to check
 * @param [in] src_idx starting search index
 * @param [in] dst_idx ending (excluded) search index
 * @param [in] epsilon optional value to use to determine zero velocities
 * @return index of the first zero velocity point found
 */
MOTION_COMMON_PUBLIC std::experimental::optional<size_t> searchZeroVelocityIndex(
  const Points & points, const size_t src_idx, const size_t dst_idx,
  const float64_t epsilon = 1e-3);

/**
 * @brief search first index with a velocity of zero in the given points
 * @param [in] points list of points to check
 */
MOTION_COMMON_PUBLIC std::experimental::optional<size_t> searchZeroVelocityIndex(
  const Points & points);

/**
 * @brief search the index of the point nearest to the given target
 * @param [in] points list of points to search
 * @param [in] point target point
 * @return index of the point nearest to the target
 */
MOTION_COMMON_PUBLIC size_t findNearestIndex(
  const Points & points,
  const geometry_msgs::msg::Point & point);

/**
 * @brief search the index of the point nearest to the given target with limits on the distance and yaw deviation
 * @param [in] points list of points to search
 * @param [in] pose target point
 * @param [in] max_dist optional maximum distance from the pose when searching for the nearest index
 * @param [in] max_yaw optional maximum deviation from the pose when searching for the nearest index
 * @return index of the point nearest to the target
 */
MOTION_COMMON_PUBLIC std::experimental::optional<size_t> findNearestIndex(
  const Points & points, const geometry_msgs::msg::Pose & pose,
  const float64_t max_dist = std::numeric_limits<float64_t>::max(),
  const float64_t max_yaw = std::numeric_limits<float64_t>::max());

/**
  * @brief calculate length along trajectory from seg_idx point to nearest point to p_target on trajectory
  *        If seg_idx point is after that nearest point, length is negative
  * @param points trajectory points
  * @param seg_idx segment index of point at beginning of length
  * @param p_target target point at end of length
  * @return signed length
  */
MOTION_COMMON_PUBLIC float64_t calcLongitudinalOffsetToSegment(
  const Points & points, const size_t seg_idx, const geometry_msgs::msg::Point & p_target);

/**
  * @brief find nearest segment index to point
  *        segment is straight path between two continuous points of trajectory
  *        When point is on a trajectory point whose index is nearest_idx, return nearest_idx - 1
  * @param points points of trajectory
  * @param point point to which to find nearest segment index
  * @return nearest index
  */
MOTION_COMMON_PUBLIC size_t findNearestSegmentIndex(
  const Points & points,
  const geometry_msgs::msg::Point & point);

/**
  * @brief calculate arc length along points
  * @param [in] points input points
  * @param [in] src_idx source index
  * @param [in] dst_idx destination index
  * @return arc length distance from source to destination along the input points
  */
MOTION_COMMON_PUBLIC float64_t calcSignedArcLength(
  const Points & points, const size_t src_idx,
  const size_t dst_idx);

/**
  * @brief calculate arc length along points
  * @param [in] points input points
  * @param [in] src_point source point
  * @param [in] dst_idx destination index
  * @return arc length distance from source to destination along the input points
  */
MOTION_COMMON_PUBLIC float64_t calcSignedArcLength(
  const Points & points, const geometry_msgs::msg::Point & src_point, const size_t & dst_idx);

/**
  * @brief calculate arc length along points
  * @param [in] points input points
  * @param [in] src_point source point
  * @param [in] dst_point destination point
  * @return arc length distance from source to destination along the input points
  */
MOTION_COMMON_PUBLIC float64_t calcSignedArcLength(
  const Points & points, const geometry_msgs::msg::Point & src_point,
  const geometry_msgs::msg::Point & dst_point);

/**
  * @brief calculate longitudinal deviation of a point relative to a pose
  * @param [in] base_pose base from which to calculate the deviation
  * @param [in] target_point point for which to calculate the deviation
  * @return longitudinal distance between the base and the target
  */
MOTION_COMMON_PUBLIC float64_t calcLongitudinalDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point);

}  // namespace motion_common
}  // namespace motion
}  // namespace autoware

#endif  // MOTION_COMMON__TRAJECTORY_COMMON_HPP_
