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

#include "motion_common/trajectory_common.hpp"

#include <limits>

#include "motion_common/motion_common.hpp"

namespace autoware
{
namespace motion
{
namespace motion_common
{
using ::motion::motion_common::to_angle;

void validateNonEmpty(const Points & points)
{
  if (points.empty()) {
    throw std::invalid_argument("Empty points");
  }
}

float64_t calcYawDeviation(const float64_t & base_yaw, const float64_t & target_yaw)
{
  return autoware::common::helper_functions::wrap_angle(target_yaw - base_yaw);
}

std::experimental::optional<size_t> searchZeroVelocityIndex(
  const Points & points, const size_t src_idx, const size_t dst_idx, const float64_t epsilon)
{
  validateNonEmpty(points);

  for (size_t i = src_idx; i < dst_idx; ++i) {
    if (static_cast<float64_t>(std::fabs(points.at(i).longitudinal_velocity_mps)) < epsilon) {
      return i;
    }
  }

  return {};
}

std::experimental::optional<size_t> searchZeroVelocityIndex(const Points & points)
{
  return searchZeroVelocityIndex(points, 0, points.size());
}

size_t findNearestIndex(const Points & points, const geometry_msgs::msg::Point & point)
{
  validateNonEmpty(points);

  float64_t min_dist = std::numeric_limits<float64_t>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist = autoware::common::geometry::distance_2d<float64_t>(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return min_idx;
}

std::experimental::optional<size_t> findNearestIndex(
  const Points & points, const geometry_msgs::msg::Pose & pose,
  const float64_t max_dist,
  const float64_t max_yaw)
{
  validateNonEmpty(points);

  float64_t min_dist = std::numeric_limits<float64_t>::max();
  bool is_nearest_found = false;
  size_t min_idx = 0;

  const auto target_yaw = to_angle(pose.orientation);
  for (size_t i = 0; i < points.size(); ++i) {
    const auto dist =
      autoware::common::geometry::distance_2d<float64_t>(points.at(i), pose.position);
    if (dist > max_dist) {
      continue;
    }

    const auto base_yaw = to_angle(points.at(i).pose.orientation);
    const auto yaw = calcYawDeviation(base_yaw, target_yaw);
    if (std::fabs(yaw) > max_yaw) {
      continue;
    }

    if (dist >= min_dist) {
      continue;
    }

    min_dist = dist;
    min_idx = i;
    is_nearest_found = true;
  }
  return is_nearest_found ? std::experimental::optional<size_t>(min_idx) : std::experimental::
         nullopt;
}

float64_t calcLongitudinalOffsetToSegment(
  const Points & points, const size_t seg_idx, const geometry_msgs::msg::Point & p_target)
{
  validateNonEmpty(points);

  const auto p_front = points.at(seg_idx);
  const auto p_back = points.at(seg_idx + 1);
  const auto x_front = static_cast<float64_t>(p_front.pose.position.x);
  const auto y_front = static_cast<float64_t>(p_front.pose.position.y);
  const auto x_back = static_cast<float64_t>(p_back.pose.position.x);
  const auto y_back = static_cast<float64_t>(p_back.pose.position.y);

  const Vector3f segment_vec{x_back - x_front, y_back - y_front, 0.0};
  const Vector3f target_vec{static_cast<float64_t>(p_target.x) - x_front,
    static_cast<float64_t>(p_target.y) - y_front, 0.0};

  if (segment_vec.norm() == 0.0) {
    throw std::runtime_error("Same points are given.");
  }

  return segment_vec.dot(target_vec) / segment_vec.norm();
}

size_t findNearestSegmentIndex(const Points & points, const geometry_msgs::msg::Point & point)
{
  const size_t nearest_idx = findNearestIndex(points, point);

  if (nearest_idx == 0) {
    return 0;
  } else if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const float64_t signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, point);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

float64_t calcSignedArcLength(const Points & points, const size_t src_idx, const size_t dst_idx)
{
  validateNonEmpty(points);

  if (src_idx > dst_idx) {
    return -calcSignedArcLength(points, dst_idx, src_idx);
  }

  float64_t dist_sum = 0.0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    dist_sum += autoware::common::geometry::distance_2d<float64_t>(points.at(i), points.at(i + 1));
  }
  return dist_sum;
}

float64_t calcSignedArcLength(
  const Points & points, const geometry_msgs::msg::Point & src_point, const size_t & dst_idx)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);

  const float64_t signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_idx);
  const float64_t signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);

  return signed_length_on_traj - signed_length_src_offset;
}

float64_t calcSignedArcLength(
  const Points & points, const geometry_msgs::msg::Point & src_point,
  const geometry_msgs::msg::Point & dst_point)
{
  validateNonEmpty(points);

  const size_t src_seg_idx = findNearestSegmentIndex(points, src_point);
  const size_t dst_seg_idx = findNearestSegmentIndex(points, dst_point);

  const float64_t signed_length_on_traj = calcSignedArcLength(points, src_seg_idx, dst_seg_idx);
  const float64_t signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_seg_idx, src_point);
  const float64_t signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_seg_idx, dst_point);

  return signed_length_on_traj - signed_length_src_offset + signed_length_dst_offset;
}

float64_t calcLongitudinalDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = base_pose.position;

  const auto yaw = to_angle(base_pose.orientation);
  const Vector3f base_unit_vec{std::cos(yaw), std::sin(yaw), 0};

  const auto dx = target_point.x - base_point.x;
  const auto dy = target_point.y - base_point.y;
  const Vector3f diff_vec{dx, dy, 0};

  return base_unit_vec.dot(diff_vec);
}
}  // namespace motion_common
}  // namespace motion
}  // namespace autoware
