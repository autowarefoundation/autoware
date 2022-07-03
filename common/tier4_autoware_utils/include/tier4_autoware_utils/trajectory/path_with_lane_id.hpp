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

#ifndef TIER4_AUTOWARE_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
#define TIER4_AUTOWARE_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/path_with_lane_id_geometry.hpp"
#include "tier4_autoware_utils/trajectory/trajectory.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace tier4_autoware_utils
{
/**
 * @brief calculate the point offset from source point along the trajectory (or path)
 * @param points points of trajectory, path, ...
 * @param src_idx index of source point
 * @param offset length of offset from source point
 * @return offset point
 */
template <>
inline boost::optional<geometry_msgs::msg::Point> calcLongitudinalOffsetPoint(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const double offset)
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
template <>
inline boost::optional<geometry_msgs::msg::Point> calcLongitudinalOffsetPoint(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const double offset)
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
template <>
inline boost::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const double offset)
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
template <>
inline boost::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const double offset)
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
template <>
inline boost::optional<size_t> insertTargetPoint(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold)
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

#endif  // TIER4_AUTOWARE_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
