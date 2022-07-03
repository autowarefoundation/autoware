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

#ifndef TIER4_AUTOWARE_UTILS__TRAJECTORY__TRAJECTORY_HPP_
#define TIER4_AUTOWARE_UTILS__TRAJECTORY__TRAJECTORY_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/pose_deviation.hpp"
#include "tier4_autoware_utils/math/constants.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

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
void validateNonSharpAngle(
  const T & point1, const T & point2, const T & point3, const double angle_threshold = pi / 4)
{
  const auto p1 = getPoint(point1);
  const auto p2 = getPoint(point2);
  const auto p3 = getPoint(point3);

  const std::vector vec_1to2 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
  const std::vector vec_3to2 = {p2.x - p3.x, p2.y - p3.y, p2.z - p3.z};
  const auto product = std::inner_product(vec_1to2.begin(), vec_1to2.end(), vec_3to2.begin(), 0.0);

  const auto dist_1to2 = calcDistance3d(p1, p2);
  const auto dist_3to2 = calcDistance3d(p3, p2);

  constexpr double epsilon = 1e-3;
  if (std::cos(angle_threshold) < product / dist_1to2 / dist_3to2 + epsilon) {
    throw std::invalid_argument("Sharp angle.");
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
 * @brief find nearest segment index to pose
 *        segment is straight path between two continuous points of trajectory
 *        When pose is on a trajectory point whose index is nearest_idx, return nearest_idx - 1
 * @param points points of trajectory
 * @param pose pose to which to find nearest segment index
 * @param max_dist max distance to search
 * @param max_yaw max yaw to search
 * @return nearest index
 */
template <class T>
boost::optional<size_t> findNearestSegmentIndex(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  const auto nearest_idx = findNearestIndex(points, pose, max_dist, max_yaw);

  if (!nearest_idx) {
    return boost::none;
  }

  if (*nearest_idx == 0) {
    return 0;
  }
  if (*nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, *nearest_idx, pose.position);

  if (signed_length <= 0) {
    return *nearest_idx - 1;
  }

  return *nearest_idx;
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
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t dst_idx)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

/**
 * @brief calcSignedArcLength from point to index with maximum distance and yaw threshold
 */
template <class T>
boost::optional<double> calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Pose & src_pose, const size_t dst_idx,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  validateNonEmpty(points);

  const auto src_seg_idx = findNearestSegmentIndex(points, src_pose, max_dist, max_yaw);
  if (!src_seg_idx) {
    return boost::none;
  }

  const double signed_length_on_traj = calcSignedArcLength(points, *src_seg_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, *src_seg_idx, src_pose.position);

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
 * @brief calcSignedArcLength from pose to point
 */
template <class T>
boost::optional<double> calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Pose & src_pose,
  const geometry_msgs::msg::Point & dst_point,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  validateNonEmpty(points);

  const auto src_seg_idx = findNearestSegmentIndex(points, src_pose, max_dist, max_yaw);
  if (!src_seg_idx) {
    return boost::none;
  }

  const size_t dst_seg_idx = findNearestSegmentIndex(points, dst_point);

  const double signed_length_on_traj = calcSignedArcLength(points, *src_seg_idx, dst_seg_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, *src_seg_idx, src_pose.position);
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

/**
 * @brief Calculate distance to the forward stop point from the given src index
 */
template <class T>
boost::optional<double> calcDistanceToForwardStopPoint(
  const T & points_with_twist, const size_t src_idx = 0)
{
  validateNonEmpty(points_with_twist);

  const auto closest_stop_idx =
    searchZeroVelocityIndex(points_with_twist, src_idx, points_with_twist.size());
  if (!closest_stop_idx) {
    return boost::none;
  }

  return std::max(0.0, calcSignedArcLength(points_with_twist, src_idx, *closest_stop_idx));
}

/**
 * @brief Calculate distance to the forward stop point from the given pose
 */
template <class T>
boost::optional<double> calcDistanceToForwardStopPoint(
  const T & points_with_twist, const geometry_msgs::msg::Pose & pose,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max())
{
  validateNonEmpty(points_with_twist);

  const auto nearest_segment_idx =
    tier4_autoware_utils::findNearestSegmentIndex(points_with_twist, pose, max_dist, max_yaw);

  if (!nearest_segment_idx) {
    return boost::none;
  }

  const auto stop_idx = tier4_autoware_utils::searchZeroVelocityIndex(
    points_with_twist, *nearest_segment_idx + 1, points_with_twist.size());

  if (!stop_idx) {
    return boost::none;
  }

  const auto closest_stop_dist = tier4_autoware_utils::calcSignedArcLength(
    points_with_twist, pose, *stop_idx, max_dist, max_yaw);

  if (!closest_stop_dist) {
    return boost::none;
  }

  return std::max(0.0, *closest_stop_dist);
}

/**
 * @brief calculate the point offset from source point along the trajectory (or path)
 * @param points points of trajectory, path, ...
 * @param src_idx index of source point
 * @param offset length of offset from source point
 * @return offset point
 */
template <class T>
inline boost::optional<geometry_msgs::msg::Point> calcLongitudinalOffsetPoint(
  const T & points, const size_t src_idx, const double offset)
{
  validateNonEmpty(points);

  if (points.size() - 1 < src_idx) {
    throw std::out_of_range("Invalid source index");
  }

  if (points.size() == 1) {
    return {};
  }

  if (src_idx + 1 == points.size() && offset == 0.0) {
    return getPoint(points.at(src_idx));
  }

  if (offset < 0.0) {
    auto reverse_points = points;
    std::reverse(reverse_points.begin(), reverse_points.end());
    return calcLongitudinalOffsetPoint(
      reverse_points, reverse_points.size() - src_idx - 1, -offset);
  }

  double dist_sum = 0.0;

  for (size_t i = src_idx; i < points.size() - 1; ++i) {
    const auto & p_front = points.at(i);
    const auto & p_back = points.at(i + 1);

    const auto dist_segment = calcDistance2d(p_front, p_back);
    dist_sum += dist_segment;

    const auto dist_res = offset - dist_sum;
    if (dist_res <= 0.0) {
      return calcInterpolatedPoint(p_back, p_front, std::abs(dist_res / dist_segment));
    }
  }

  // not found (out of range)
  return {};
}

/**
 * @brief calculate the point offset from source point along the trajectory (or path)
 * @param points points of trajectory, path, ...
 * @param src_point source point
 * @param offset length of offset from source point
 * @return offset point
 */
template <class T>
inline boost::optional<geometry_msgs::msg::Point> calcLongitudinalOffsetPoint(
  const T & points, const geometry_msgs::msg::Point & src_point, const double offset)
{
  validateNonEmpty(points);

  if (offset < 0.0) {
    auto reverse_points = points;
    std::reverse(reverse_points.begin(), reverse_points.end());
    return calcLongitudinalOffsetPoint(reverse_points, src_point, -offset);
  }

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return calcLongitudinalOffsetPoint(points, src_seg_idx, offset + signed_length_src_offset);
}

/**
 * @brief calculate the point offset from source point along the trajectory (or path)
 * @param points points of trajectory, path, ...
 * @param src_idx index of source point
 * @param offset length of offset from source point
 * @return offset pose
 */
template <class T>
inline boost::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const T & points, const size_t src_idx, const double offset)
{
  validateNonEmpty(points);

  if (points.size() - 1 < src_idx) {
    throw std::out_of_range("Invalid source index");
  }

  if (points.size() == 1) {
    return {};
  }

  if (src_idx + 1 == points.size() && offset == 0.0) {
    return getPose(points.at(src_idx));
  }

  if (offset < 0.0) {
    auto reverse_points = points;
    std::reverse(reverse_points.begin(), reverse_points.end());

    double dist_sum = 0.0;

    for (size_t i = reverse_points.size() - src_idx - 1; i < reverse_points.size() - 1; ++i) {
      const auto & p_front = reverse_points.at(i);
      const auto & p_back = reverse_points.at(i + 1);

      const auto dist_segment = calcDistance2d(p_front, p_back);
      dist_sum += dist_segment;

      const auto dist_res = -offset - dist_sum;
      if (dist_res <= 0.0) {
        return calcInterpolatedPose(p_back, p_front, std::abs(dist_res / dist_segment));
      }
    }
  } else {
    double dist_sum = 0.0;

    for (size_t i = src_idx; i < points.size() - 1; ++i) {
      const auto & p_front = points.at(i);
      const auto & p_back = points.at(i + 1);

      const auto dist_segment = calcDistance2d(p_front, p_back);
      dist_sum += dist_segment;

      const auto dist_res = offset - dist_sum;
      if (dist_res <= 0.0) {
        return calcInterpolatedPose(p_front, p_back, 1.0 - std::abs(dist_res / dist_segment));
      }
    }
  }

  // not found (out of range)
  return {};
}

/**
 * @brief calculate the point offset from source point along the trajectory (or path)
 * @param points points of trajectory, path, ...
 * @param src_point source point
 * @param offset length of offset from source point
 * @return offset pase
 */
template <class T>
inline boost::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const T & points, const geometry_msgs::msg::Point & src_point, const double offset)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return calcLongitudinalOffsetPose(points, src_seg_idx, offset + signed_length_src_offset);
}

/**
 * @brief calculate the point offset from source point along the trajectory (or path)
 * @param seg_idx segment index of point at beginning of length
 * @param p_target point to be inserted
 * @param points output points of trajectory, path, ...
 * @return index of insert point
 */
template <class T>
inline boost::optional<size_t> insertTargetPoint(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target, T & points,
  const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points);

  // invalid segment index
  if (seg_idx + 1 >= points.size()) {
    return {};
  }

  const auto p_front = getPoint(points.at(seg_idx));
  const auto p_back = getPoint(points.at(seg_idx + 1));

  try {
    validateNonSharpAngle(p_front, p_target, p_back);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  const auto overlap_with_front = calcDistance2d(p_target, p_front) < overlap_threshold;
  const auto overlap_with_back = calcDistance2d(p_target, p_back) < overlap_threshold;

  geometry_msgs::msg::Pose target_pose;
  {
    const auto pitch = calcElevationAngle(p_target, p_back);
    const auto yaw = calcAzimuthAngle(p_target, p_back);

    target_pose.position = p_target;
    target_pose.orientation = createQuaternionFromRPY(0.0, pitch, yaw);
  }

  auto p_insert = points.at(seg_idx);
  setPose(target_pose, p_insert);

  geometry_msgs::msg::Pose front_pose;
  {
    const auto pitch = calcElevationAngle(p_front, p_target);
    const auto yaw = calcAzimuthAngle(p_front, p_target);

    front_pose.position = getPoint(points.at(seg_idx));
    front_pose.orientation = createQuaternionFromRPY(0.0, pitch, yaw);
  }

  if (!overlap_with_front && !overlap_with_back) {
    setPose(front_pose, points.at(seg_idx));
    points.insert(points.begin() + seg_idx + 1, p_insert);
    return seg_idx + 1;
  }

  if (overlap_with_back) {
    return seg_idx + 1;
  }

  return seg_idx;
}
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__TRAJECTORY__TRAJECTORY_HPP_
