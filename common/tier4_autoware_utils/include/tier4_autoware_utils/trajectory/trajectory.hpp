// Copyright 2021 Tier IV, Inc.
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

#ifndef TIER4_AUTOWARE_UTILS__TRAJECTORY__TRAJECTORY_HPP_
#define TIER4_AUTOWARE_UTILS__TRAJECTORY__TRAJECTORY_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/pose_deviation.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace tier4_autoware_utils
{
template <class T>
void validateNonEmpty(const T & points)
{
  if (points.empty()) {
    throw std::invalid_argument("Points is empty.");
  }
}

template <class T>
boost::optional<size_t> searchZeroVelocityIndex(
  const T & points_with_twist, const size_t src_idx, const size_t dst_idx)
{
  validateNonEmpty(points_with_twist);

  constexpr double epsilon = 1e-3;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    if (std::fabs(points_with_twist.at(i).longitudinal_velocity_mps) < epsilon) {
      return i;
    }
  }

  return {};
}

template <class T>
boost::optional<size_t> searchZeroVelocityIndex(const T & points_with_twist)
{
  return searchZeroVelocityIndex(points_with_twist, 0, points_with_twist.size());
}

template <class T>
size_t findNearestIndex(const T & points, const geometry_msgs::msg::Point & point)
{
  validateNonEmpty(points);

  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = calcSquaredDistance2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

template <class T>
boost::optional<size_t> findNearestIndex(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  validateNonEmpty(points);

  const double max_squared_dist = max_dist * max_dist;

  double min_squared_dist = std::numeric_limits<double>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto squared_dist = calcSquaredDistance2d(points.at(i), pose);
    if (squared_dist > max_squared_dist) {
      continue;
    }

    const auto yaw = calcYawDeviation(getPose(points.at(i)), pose);
    if (std::fabs(yaw) > max_yaw) {
      continue;
    }

    if (squared_dist >= min_squared_dist) {
      continue;
    }

    min_squared_dist = squared_dist;
    min_idx = i;
    is_nearest_found = true;
  }
  return is_nearest_found ? boost::optional<size_t>(min_idx) : boost::none;
}

/**
 * @brief calculate longitudinal offset (length along trajectory from seg_idx point to nearest point
 * to p_target on trajectory) If seg_idx point is after that nearest point, length is negative
 * @param points points of trajectory, path, ...
 * @param seg_idx segment index of point at beginning of length
 * @param p_target target point at end of length
 * @return signed length
 */
template <class T>
double calcLongitudinalOffsetToSegment(
  const T & points, const size_t seg_idx, const geometry_msgs::msg::Point & p_target)
{
  validateNonEmpty(points);

  const auto p_front = getPoint(points.at(seg_idx));
  const auto p_back = getPoint(points.at(seg_idx + 1));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0};

  if (segment_vec.norm() == 0.0) {
    throw std::runtime_error("Same points are given.");
  }

  return segment_vec.dot(target_vec) / segment_vec.norm();
}

/**
 * @brief find nearest segment index to point
 *        segment is straight path between two continuous points of trajectory
 *        When point is on a trajectory point whose index is nearest_idx, return nearest_idx - 1
 * @param points points of trajectory
 * @param point point to which to find nearest segment index
 * @return nearest index
 */
template <class T>
size_t findNearestSegmentIndex(const T & points, const geometry_msgs::msg::Point & point)
{
  const size_t nearest_idx = findNearestIndex(points, point);

  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, point);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

/**
 * @brief calculate lateral offset from p_target (length from p_target to trajectory)
 *        If seg_idx point is after that nearest point, length is negative
 * @param points points of trajectory, path, ...
 * @param p_target target point
 * @return length (unsigned)
 */
template <class T>
double calcLateralOffset(const T & points, const geometry_msgs::msg::Point & p_target)
{
  validateNonEmpty(points);

  const size_t seg_idx = findNearestSegmentIndex(points, p_target);

  const auto p_front = getPoint(points.at(seg_idx));
  const auto p_back = getPoint(points.at(seg_idx + 1));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0.0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0.0};

  if (segment_vec.norm() == 0.0) {
    throw std::runtime_error("Same points are given.");
  }

  const Eigen::Vector3d cross_vec = segment_vec.cross(target_vec);
  return cross_vec(2) / segment_vec.norm();
}

/**
 * @brief calcSignedArcLength from index to index
 */
template <class T>
double calcSignedArcLength(const T & points, const size_t src_idx, const size_t dst_idx)
{
  validateNonEmpty(points);

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  double dist_sum = 0.0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += calcDistance2d(points.at(i), points.at(i + 1));
  }
  return dist_sum;
}

/**
 * @brief calcSignedArcLength from point to index
 */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t & dst_idx)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

/**
 * @brief calcSignedArcLength from index to point
 */
template <class T>
double calcSignedArcLength(
  const T & points, const size_t src_idx, const geometry_msgs::msg::Point & dst_point)
{
  validateNonEmpty(points);

  return -calcSignedArcLength(points, dst_point, src_idx);
}

/**
 * @brief calcSignedArcLength from point to point
 */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point,
  const geometry_msgs::msg::Point & dst_point)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const size_t dst_seg_idx = findNearestSegmentIndex(points, dst_point);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}

/**
 * @brief calcArcLength for the whole length
 */
template <class T>
double calcArcLength(const T & points)
{
  validateNonEmpty(points);

  return calcSignedArcLength(points, 0, points.size() - 1);
}
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__TRAJECTORY__TRAJECTORY_HPP_
