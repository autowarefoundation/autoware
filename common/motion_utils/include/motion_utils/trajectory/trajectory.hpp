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

#ifndef MOTION_UTILS__TRAJECTORY__TRAJECTORY_HPP_
#define MOTION_UTILS__TRAJECTORY__TRAJECTORY_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/pose_deviation.hpp"
#include "tier4_autoware_utils/math/constants.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace motion_utils
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
  const T & point1, const T & point2, const T & point3,
  const double angle_threshold = tier4_autoware_utils::pi / 4)
{
  const auto p1 = tier4_autoware_utils::getPoint(point1);
  const auto p2 = tier4_autoware_utils::getPoint(point2);
  const auto p3 = tier4_autoware_utils::getPoint(point3);

  const std::vector vec_1to2 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
  const std::vector vec_3to2 = {p2.x - p3.x, p2.y - p3.y, p2.z - p3.z};
  const auto product = std::inner_product(vec_1to2.begin(), vec_1to2.end(), vec_3to2.begin(), 0.0);

  const auto dist_1to2 = tier4_autoware_utils::calcDistance3d(p1, p2);
  const auto dist_3to2 = tier4_autoware_utils::calcDistance3d(p3, p2);

  constexpr double epsilon = 1e-3;
  if (std::cos(angle_threshold) < product / dist_1to2 / dist_3to2 + epsilon) {
    throw std::invalid_argument("Sharp angle.");
  }
}

template <class T>
boost::optional<bool> isDrivingForward(const T points)
{
  if (points.size() < 2) {
    return boost::none;
  }

  // check the first point direction
  const auto & first_pose = tier4_autoware_utils::getPose(points.at(0));
  const auto & second_pose = tier4_autoware_utils::getPose(points.at(1));

  return tier4_autoware_utils::isDrivingForward(first_pose, second_pose);
}

template <class T>
boost::optional<bool> isDrivingForwardWithTwist(const T points_with_twist)
{
  if (points_with_twist.empty()) {
    return boost::none;
  }
  if (points_with_twist.size() == 1) {
    if (0.0 < tier4_autoware_utils::getLongitudinalVelocity(points_with_twist.front())) {
      return true;
    } else if (0.0 > tier4_autoware_utils::getLongitudinalVelocity(points_with_twist.front())) {
      return false;
    } else {
      return boost::none;
    }
  }

  return isDrivingForward(points_with_twist);
}

template <class T>
T removeOverlapPoints(const T & points, const size_t & start_idx = 0)
{
  if (points.size() < start_idx + 1) {
    return points;
  }

  T dst;

  for (size_t i = 0; i <= start_idx; ++i) {
    dst.push_back(points.at(i));
  }

  constexpr double eps = 1.0E-08;
  for (size_t i = start_idx + 1; i < points.size(); ++i) {
    const auto prev_p = tier4_autoware_utils::getPoint(dst.back());
    const auto curr_p = tier4_autoware_utils::getPoint(points.at(i));
    const double dist = tier4_autoware_utils::calcDistance2d(prev_p, curr_p);
    if (dist < eps) {
      continue;
    }
    dst.push_back(points.at(i));
  }

  return dst;
}

template <class T>
boost::optional<size_t> searchZeroVelocityIndex(
  const T & points_with_twist, const size_t src_idx, const size_t dst_idx)
{
  try {
    validateNonEmpty(points_with_twist);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  constexpr double epsilon = 1e-3;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    if (std::fabs(points_with_twist.at(i).longitudinal_velocity_mps) < epsilon) {
      return i;
    }
  }

  return {};
}

template <class T>
boost::optional<size_t> searchZeroVelocityIndex(const T & points_with_twist, const size_t & src_idx)
{
  try {
    validateNonEmpty(points_with_twist);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  return searchZeroVelocityIndex(points_with_twist, src_idx, points_with_twist.size());
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
    const auto dist = tier4_autoware_utils::calcSquaredDistance2d(points.at(i), point);
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
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  const double max_squared_dist = max_dist * max_dist;

  double min_squared_dist = std::numeric_limits<double>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto squared_dist = tier4_autoware_utils::calcSquaredDistance2d(points.at(i), pose);
    if (squared_dist > max_squared_dist) {
      continue;
    }

    const auto yaw =
      tier4_autoware_utils::calcYawDeviation(tier4_autoware_utils::getPose(points.at(i)), pose);
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
  const T & points, const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  const bool throw_exception = false)
{
  if (seg_idx >= points.size() - 1) {
    const std::out_of_range e("Segment index is invalid.");
    if (throw_exception) {
      throw e;
    }
    std::cerr << e.what() << std::endl;
    return std::nan("");
  }

  const auto overlap_removed_points = removeOverlapPoints(points, seg_idx);

  if (throw_exception) {
    validateNonEmpty(overlap_removed_points);
  } else {
    try {
      validateNonEmpty(overlap_removed_points);
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
      return std::nan("");
    }
  }

  if (seg_idx >= overlap_removed_points.size() - 1) {
    const std::runtime_error e("Same points are given.");
    if (throw_exception) {
      throw e;
    }
    std::cerr << e.what() << std::endl;
    return std::nan("");
  }

  const auto p_front = tier4_autoware_utils::getPoint(overlap_removed_points.at(seg_idx));
  const auto p_back = tier4_autoware_utils::getPoint(overlap_removed_points.at(seg_idx + 1));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0};

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
double calcLateralOffset(
  const T & points, const geometry_msgs::msg::Point & p_target, const size_t seg_idx,
  const bool throw_exception = false)
{
  const auto overlap_removed_points = removeOverlapPoints(points, 0);

  if (throw_exception) {
    validateNonEmpty(overlap_removed_points);
  } else {
    try {
      validateNonEmpty(overlap_removed_points);
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
      return std::nan("");
    }
  }

  if (overlap_removed_points.size() == 1) {
    const std::runtime_error e("Same points are given.");
    if (throw_exception) {
      throw e;
    }
    std::cerr << e.what() << std::endl;
    return std::nan("");
  }

  const auto p_front = tier4_autoware_utils::getPoint(overlap_removed_points.at(seg_idx));
  const auto p_back = tier4_autoware_utils::getPoint(overlap_removed_points.at(seg_idx + 1));

  const Eigen::Vector3d segment_vec{p_back.x - p_front.x, p_back.y - p_front.y, 0.0};
  const Eigen::Vector3d target_vec{p_target.x - p_front.x, p_target.y - p_front.y, 0.0};

  const Eigen::Vector3d cross_vec = segment_vec.cross(target_vec);
  return cross_vec(2) / segment_vec.norm();
}

template <class T>
double calcLateralOffset(
  const T & points, const geometry_msgs::msg::Point & p_target, const bool throw_exception = false)
{
  const auto overlap_removed_points = removeOverlapPoints(points, 0);

  if (throw_exception) {
    validateNonEmpty(overlap_removed_points);
  } else {
    try {
      validateNonEmpty(overlap_removed_points);
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
      return std::nan("");
    }
  }

  if (overlap_removed_points.size() == 1) {
    const std::runtime_error e("Same points are given.");
    if (throw_exception) {
      throw e;
    }
    std::cerr << e.what() << std::endl;
    return std::nan("");
  }

  const size_t seg_idx = findNearestSegmentIndex(overlap_removed_points, p_target);
  return calcLateralOffset(points, p_target, seg_idx, throw_exception);
}

/**
 * @brief calcSignedArcLength from index to index
 */
template <class T>
double calcSignedArcLength(const T & points, const size_t src_idx, const size_t dst_idx)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 0.0;
  }

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  double dist_sum = 0.0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
  }
  return dist_sum;
}

/**
 *  @brief Computes the partial sums of the elements in the sub-ranges of
 *         the range [src_idx, dst_idx) and return these sum as vector
 */
template <class T>
std::vector<double> calcSignedArcLengthPartialSum(
  const T & points, const size_t src_idx, const size_t dst_idx)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  if (src_idx + 1 > dst_idx) {
    auto copied = points;
    std::reverse(copied.begin(), copied.end());
    return calcSignedArcLengthPartialSum(points, dst_idx, src_idx);
  }

  std::vector<double> partial_dist;
  partial_dist.reserve(dst_idx - src_idx);

  double dist_sum = 0.0;
  partial_dist.push_back(dist_sum);
  for (size_t i = src_idx; i < dst_idx - 1; ++i) {
    dist_sum += tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
    partial_dist.push_back(dist_sum);
  }
  return partial_dist;
}

/**
 * @brief calcSignedArcLength from point to index
 */
template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t dst_idx)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 0.0;
  }

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
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

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
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 0.0;
  }

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
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 0.0;
  }

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
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

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
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return 0.0;
  }

  return calcSignedArcLength(points, 0, points.size() - 1);
}

template <class T>
inline std::vector<double> calcCurvature(const T & points)
{
  std::vector<double> curvature_vec(points.size());

  for (size_t i = 1; i < points.size() - 1; ++i) {
    const auto p1 = tier4_autoware_utils::getPoint(points.at(i - 1));
    const auto p2 = tier4_autoware_utils::getPoint(points.at(i));
    const auto p3 = tier4_autoware_utils::getPoint(points.at(i + 1));
    curvature_vec.at(i) = (tier4_autoware_utils::calcCurvature(p1, p2, p3));
  }
  curvature_vec.at(0) = curvature_vec.at(1);
  curvature_vec.at(curvature_vec.size() - 1) = curvature_vec.at(curvature_vec.size() - 2);

  return curvature_vec;
}

template <class T>
inline std::vector<std::pair<double, double>> calcCurvatureAndArcLength(const T & points)
{
  // Note that arclength is for the segment, not the sum.
  std::vector<std::pair<double, double>> curvature_arc_length_vec;
  curvature_arc_length_vec.push_back(std::pair(0.0, 0.0));
  for (size_t i = 1; i < points.size() - 1; ++i) {
    const auto p1 = tier4_autoware_utils::getPoint(points.at(i - 1));
    const auto p2 = tier4_autoware_utils::getPoint(points.at(i));
    const auto p3 = tier4_autoware_utils::getPoint(points.at(i + 1));
    const double curvature = tier4_autoware_utils::calcCurvature(p1, p2, p3);
    const double arc_length = tier4_autoware_utils::calcDistance2d(points.at(i - 1), points.at(i)) +
                              tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
    curvature_arc_length_vec.push_back(std::pair(curvature, arc_length));
  }
  curvature_arc_length_vec.push_back(std::pair(0.0, 0.0));

  return curvature_arc_length_vec;
}

/**
 * @brief Calculate distance to the forward stop point from the given src index
 */
template <class T>
boost::optional<double> calcDistanceToForwardStopPoint(
  const T & points_with_twist, const size_t src_idx = 0)
{
  try {
    validateNonEmpty(points_with_twist);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

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
  try {
    validateNonEmpty(points_with_twist);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  const auto nearest_segment_idx =
    motion_utils::findNearestSegmentIndex(points_with_twist, pose, max_dist, max_yaw);

  if (!nearest_segment_idx) {
    return boost::none;
  }

  const auto stop_idx = motion_utils::searchZeroVelocityIndex(
    points_with_twist, *nearest_segment_idx + 1, points_with_twist.size());

  if (!stop_idx) {
    return boost::none;
  }

  const auto closest_stop_dist =
    motion_utils::calcSignedArcLength(points_with_twist, pose, *stop_idx, max_dist, max_yaw);

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
  const T & points, const size_t src_idx, const double offset, const bool throw_exception = false)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  if (points.size() - 1 < src_idx) {
    const auto e = std::out_of_range("Invalid source index");
    if (throw_exception) {
      throw e;
    }
    std::cerr << e.what() << std::endl;
    return {};
  }

  if (points.size() == 1) {
    return {};
  }

  if (src_idx + 1 == points.size() && offset == 0.0) {
    return tier4_autoware_utils::getPoint(points.at(src_idx));
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

    const auto dist_segment = tier4_autoware_utils::calcDistance2d(p_front, p_back);
    dist_sum += dist_segment;

    const auto dist_res = offset - dist_sum;
    if (dist_res <= 0.0) {
      return tier4_autoware_utils::calcInterpolatedPoint(
        p_back, p_front, std::abs(dist_res / dist_segment));
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
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

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
 * @param set_orientation_from_position_direction set orientation by spherical interpolation if
 * false
 * @return offset pose
 */
template <class T>
inline boost::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const T & points, const size_t src_idx, const double offset,
  const bool set_orientation_from_position_direction = true, const bool throw_exception = false)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  if (points.size() - 1 < src_idx) {
    const auto e = std::out_of_range("Invalid source index");
    if (throw_exception) {
      throw e;
    }
    std::cerr << e.what() << std::endl;
    return {};
  }

  if (points.size() == 1) {
    return {};
  }

  if (src_idx + 1 == points.size() && offset == 0.0) {
    return tier4_autoware_utils::getPose(points.at(src_idx));
  }

  if (offset < 0.0) {
    auto reverse_points = points;
    std::reverse(reverse_points.begin(), reverse_points.end());

    double dist_sum = 0.0;

    for (size_t i = reverse_points.size() - src_idx - 1; i < reverse_points.size() - 1; ++i) {
      const auto & p_front = reverse_points.at(i);
      const auto & p_back = reverse_points.at(i + 1);

      const auto dist_segment = tier4_autoware_utils::calcDistance2d(p_front, p_back);
      dist_sum += dist_segment;

      const auto dist_res = -offset - dist_sum;
      if (dist_res <= 0.0) {
        return tier4_autoware_utils::calcInterpolatedPose(
          p_back, p_front, std::abs(dist_res / dist_segment),
          set_orientation_from_position_direction);
      }
    }
  } else {
    double dist_sum = 0.0;

    for (size_t i = src_idx; i < points.size() - 1; ++i) {
      const auto & p_front = points.at(i);
      const auto & p_back = points.at(i + 1);

      const auto dist_segment = tier4_autoware_utils::calcDistance2d(p_front, p_back);
      dist_sum += dist_segment;

      const auto dist_res = offset - dist_sum;
      if (dist_res <= 0.0) {
        return tier4_autoware_utils::calcInterpolatedPose(
          p_front, p_back, 1.0 - std::abs(dist_res / dist_segment),
          set_orientation_from_position_direction);
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
 * @param set_orientation_from_position_direction set orientation by spherical interpolation if
 * false
 * @return offset pase
 */
template <class T>
inline boost::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const T & points, const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction = true)
{
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return calcLongitudinalOffsetPose(
    points, src_seg_idx, offset + signed_length_src_offset,
    set_orientation_from_position_direction);
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
  try {
    validateNonEmpty(points);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  // invalid segment index
  if (seg_idx + 1 >= points.size()) {
    return {};
  }

  const auto p_front = tier4_autoware_utils::getPoint(points.at(seg_idx));
  const auto p_back = tier4_autoware_utils::getPoint(points.at(seg_idx + 1));

  try {
    validateNonSharpAngle(p_front, p_target, p_back);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
    return {};
  }

  const auto overlap_with_front =
    tier4_autoware_utils::calcDistance2d(p_target, p_front) < overlap_threshold;
  const auto overlap_with_back =
    tier4_autoware_utils::calcDistance2d(p_target, p_back) < overlap_threshold;

  const auto is_driving_forward = isDrivingForward(points);
  if (!is_driving_forward) {
    return {};
  }

  geometry_msgs::msg::Pose target_pose;
  {
    const auto p_base = is_driving_forward.get() ? p_back : p_front;
    const auto pitch = tier4_autoware_utils::calcElevationAngle(p_target, p_base);
    const auto yaw = tier4_autoware_utils::calcAzimuthAngle(p_target, p_base);

    target_pose.position = p_target;
    target_pose.orientation = tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw);
  }

  auto p_insert = points.at(seg_idx);
  tier4_autoware_utils::setPose(target_pose, p_insert);

  geometry_msgs::msg::Pose base_pose;
  {
    const auto p_base = is_driving_forward.get() ? p_front : p_back;
    const auto pitch = tier4_autoware_utils::calcElevationAngle(p_base, p_target);
    const auto yaw = tier4_autoware_utils::calcAzimuthAngle(p_base, p_target);

    base_pose.position = tier4_autoware_utils::getPoint(p_base);
    base_pose.orientation = tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw);
  }

  if (!overlap_with_front && !overlap_with_back) {
    if (is_driving_forward.get()) {
      tier4_autoware_utils::setPose(base_pose, points.at(seg_idx));
    } else {
      tier4_autoware_utils::setPose(base_pose, points.at(seg_idx + 1));
    }
    points.insert(points.begin() + seg_idx + 1, p_insert);
    return seg_idx + 1;
  }

  if (overlap_with_back) {
    return seg_idx + 1;
  }

  return seg_idx;
}

/**
 * @brief calculate the point offset from source point along the trajectory (or path)
 * @param insert_point_length length to insert point from the beginning of the points
 * @param p_target point to be inserted
 * @param points output points of trajectory, path, ...
 * @return index of insert point
 */
template <class T>
inline boost::optional<size_t> insertTargetPoint(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target, T & points,
  const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points);

  if (insert_point_length < 0.0) {
    return boost::none;
  }

  // Get Nearest segment index
  boost::optional<size_t> segment_idx = boost::none;
  for (size_t i = 1; i < points.size(); ++i) {
    const double length = calcSignedArcLength(points, 0, i);
    if (insert_point_length <= length) {
      segment_idx = i - 1;
      break;
    }
  }

  if (!segment_idx) {
    return boost::none;
  }

  return insertTargetPoint(*segment_idx, p_target, points, overlap_threshold);
}

/**
 * @brief Insert a target point from the source index
 * @param src_segment_idx source segment index on the trajectory
 * @param insert_point_length length to insert point from the beginning of the points
 * @param points output points of trajectory, path, ...
 * @return index of insert point
 */
template <class T>
inline boost::optional<size_t> insertTargetPoint(
  const size_t src_segment_idx, const double insert_point_length, T & points,
  const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points);

  if (insert_point_length < 0.0 || src_segment_idx >= points.size() - 1) {
    return boost::none;
  }

  // Get Nearest segment index
  boost::optional<size_t> segment_idx = boost::none;
  for (size_t i = src_segment_idx + 1; i < points.size(); ++i) {
    const double length = calcSignedArcLength(points, src_segment_idx, i);
    if (insert_point_length <= length) {
      segment_idx = i - 1;
      break;
    }
  }

  if (!segment_idx) {
    return boost::none;
  }

  // Get Target Point
  const double segment_length = calcSignedArcLength(points, *segment_idx, *segment_idx + 1);
  const double target_length =
    std::max(0.0, insert_point_length - calcSignedArcLength(points, src_segment_idx, *segment_idx));
  const double ratio = std::clamp(target_length / segment_length, 0.0, 1.0);
  const auto p_target = tier4_autoware_utils::calcInterpolatedPoint(
    tier4_autoware_utils::getPoint(points.at(*segment_idx)),
    tier4_autoware_utils::getPoint(points.at(*segment_idx + 1)), ratio);

  return insertTargetPoint(*segment_idx, p_target, points, overlap_threshold);
}

/**
 * @brief Insert a target point from a source pose on the trajectory
 * @param src_pose source pose on the trajectory
 * @param insert_point_length length to insert point from the beginning of the points
 * @param points output points of trajectory, path, ...
 * @return index of insert point
 */
template <class T>
inline boost::optional<size_t> insertTargetPoint(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length, T & points,
  const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points);

  if (insert_point_length < 0.0) {
    return boost::none;
  }

  const auto nearest_segment_idx = findNearestSegmentIndex(points, src_pose, max_dist, max_yaw);
  if (!nearest_segment_idx) {
    return boost::none;
  }

  const double offset_length =
    calcLongitudinalOffsetToSegment(points, *nearest_segment_idx, src_pose.position);

  return insertTargetPoint(
    *nearest_segment_idx, insert_point_length + offset_length, points, overlap_threshold);
}

/**
 * @brief Insert stop point from the source segment index
 * @param src_segment_idx start segment index on the trajectory
 * @param distance_to_stop_point distance to stop point from the source index
 * @param points output points of trajectory, path, ...
 * @return index of stop point
 */
template <class T>
inline boost::optional<size_t> insertStopPoint(
  const size_t src_segment_idx, const double distance_to_stop_point, T & points_with_twist,
  const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points_with_twist);

  if (distance_to_stop_point < 0.0 || src_segment_idx >= points_with_twist.size() - 1) {
    return boost::none;
  }

  const auto stop_idx = insertTargetPoint(
    src_segment_idx, distance_to_stop_point, points_with_twist, overlap_threshold);
  if (!stop_idx) {
    return boost::none;
  }

  for (size_t i = *stop_idx; i < points_with_twist.size(); ++i) {
    tier4_autoware_utils::setLongitudinalVelocity(0.0, points_with_twist.at(i));
  }

  return stop_idx;
}

/**
 * @brief Insert Stop point from the source pose
 * @param src_pose source pose
 * @param distance_to_stop_point  distance to stop point from the src point
 * @param points output points of trajectory, path, ...
 * @return index of stop point
 */
template <class T>
inline boost::optional<size_t> insertStopPoint(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  T & points_with_twist, const double max_dist = std::numeric_limits<double>::max(),
  const double max_yaw = std::numeric_limits<double>::max(), const double overlap_threshold = 1e-3)
{
  validateNonEmpty(points_with_twist);

  if (distance_to_stop_point < 0.0) {
    return boost::none;
  }

  const auto stop_idx = insertTargetPoint(
    src_pose, distance_to_stop_point, points_with_twist, max_dist, max_yaw, overlap_threshold);

  if (!stop_idx) {
    return boost::none;
  }

  for (size_t i = *stop_idx; i < points_with_twist.size(); ++i) {
    tier4_autoware_utils::setLongitudinalVelocity(0.0, points_with_twist.at(i));
  }

  return stop_idx;
}

template <class T>
void insertOrientation(T & points, const bool is_driving_forward)
{
  if (is_driving_forward) {
    for (size_t i = 0; i < points.size() - 1; ++i) {
      const auto & src_point = tier4_autoware_utils::getPoint(points.at(i));
      const auto & dst_point = tier4_autoware_utils::getPoint(points.at(i + 1));
      const double pitch = tier4_autoware_utils::calcElevationAngle(src_point, dst_point);
      const double yaw = tier4_autoware_utils::calcAzimuthAngle(src_point, dst_point);
      tier4_autoware_utils::setOrientation(
        tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw), points.at(i));
      if (i == points.size() - 2) {
        // Terminal orientation is same as the point before it
        tier4_autoware_utils::setOrientation(
          tier4_autoware_utils::getPose(points.at(i)).orientation, points.at(i + 1));
      }
    }
  } else {
    for (size_t i = points.size() - 1; i >= 1; --i) {
      const auto & src_point = tier4_autoware_utils::getPoint(points.at(i));
      const auto & dst_point = tier4_autoware_utils::getPoint(points.at(i - 1));
      const double pitch = tier4_autoware_utils::calcElevationAngle(src_point, dst_point);
      const double yaw = tier4_autoware_utils::calcAzimuthAngle(src_point, dst_point);
      tier4_autoware_utils::setOrientation(
        tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw), points.at(i));
    }
    // Initial orientation is same as the point after it
    tier4_autoware_utils::setOrientation(
      tier4_autoware_utils::getPose(points.at(1)).orientation, points.at(0));
  }
}

template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx)
{
  validateNonEmpty(points);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}

template <class T>
double calcSignedArcLength(
  const T & points, const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const size_t dst_idx)
{
  validateNonEmpty(points);

  const double signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

template <class T>
double calcSignedArcLength(
  const T & points, const size_t src_idx, const geometry_msgs::msg::Point & dst_point,
  const size_t dst_seg_idx)
{
  validateNonEmpty(points);

  const double signed_length_on_traj = calcSignedArcLength(points, src_idx, dst_seg_idx);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj + signed_length_dst_offset;
}

template <class T>
size_t findFirstNearestIndexWithSoftConstraints(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  validateNonEmpty(points);

  {  // with dist and yaw thresholds
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto squared_dist =
        tier4_autoware_utils::calcSquaredDistance2d(points.at(i), pose.position);
      const auto yaw =
        tier4_autoware_utils::calcYawDeviation(tier4_autoware_utils::getPose(points.at(i)), pose);

      if (squared_dist_threshold < squared_dist || yaw_threshold < std::abs(yaw)) {
        if (is_within_constraints) {
          break;
        } else {
          continue;
        }
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  {  // with dist threshold
    const double squared_dist_threshold = dist_threshold * dist_threshold;
    double min_squared_dist = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    bool is_within_constraints = false;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto squared_dist =
        tier4_autoware_utils::calcSquaredDistance2d(points.at(i), pose.position);

      if (squared_dist_threshold < squared_dist) {
        if (is_within_constraints) {
          break;
        } else {
          continue;
        }
      }

      if (min_squared_dist <= squared_dist) {
        continue;
      }

      min_squared_dist = squared_dist;
      min_idx = i;
      is_within_constraints = true;
    }

    // nearest index is found
    if (is_within_constraints) {
      return min_idx;
    }
  }

  // without any threshold
  return findNearestIndex(points, pose.position);
}

template <class T>
size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const T & points, const geometry_msgs::msg::Pose & pose,
  const double dist_threshold = std::numeric_limits<double>::max(),
  const double yaw_threshold = std::numeric_limits<double>::max())
{
  // find first nearest index with soft constraints (not segment index)
  const size_t nearest_idx =
    findFirstNearestIndexWithSoftConstraints(points, pose, dist_threshold, yaw_threshold);

  // calculate segment index
  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, pose.position);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}
}  // namespace motion_utils

#endif  // MOTION_UTILS__TRAJECTORY__TRAJECTORY_HPP_
