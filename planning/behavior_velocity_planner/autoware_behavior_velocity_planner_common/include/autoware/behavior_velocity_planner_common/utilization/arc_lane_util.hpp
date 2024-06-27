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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__ARC_LANE_UTIL_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__ARC_LANE_UTIL_HPP_

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <algorithm>
#include <optional>
#include <utility>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

namespace autoware::behavior_velocity_planner
{
namespace
{
geometry_msgs::msg::Point convertToGeomPoint(const autoware::universe_utils::Point2d & p)
{
  geometry_msgs::msg::Point geom_p;
  geom_p.x = p.x();
  geom_p.y = p.y();

  return geom_p;
}

}  // namespace

namespace arc_lane_utils
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;  // front index, pose
using PathIndexWithPoint2d =
  std::pair<size_t, autoware::universe_utils::Point2d>;                   // front index, point2d
using PathIndexWithPoint = std::pair<size_t, geometry_msgs::msg::Point>;  // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                    // front index, offset

double calcSignedDistance(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Point & p2);

// calculate one collision point between the line (from p1 to p2) and the line (from p3 to p4)
std::optional<geometry_msgs::msg::Point> checkCollision(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4);

template <class T>
std::optional<PathIndexWithPoint> findCollisionSegment(
  const T & path, const geometry_msgs::msg::Point & stop_line_p1,
  const geometry_msgs::msg::Point & stop_line_p2)
{
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto & p1 =
      autoware::universe_utils::getPoint(path.points.at(i));  // Point before collision point
    const auto & p2 =
      autoware::universe_utils::getPoint(path.points.at(i + 1));  // Point after collision point

    const auto collision_point = checkCollision(p1, p2, stop_line_p1, stop_line_p2);

    if (collision_point) {
      return std::make_pair(i, collision_point.value());
    }
  }

  return {};
}

template <class T>
std::optional<PathIndexWithPoint> findCollisionSegment(
  const T & path, const LineString2d & stop_line)
{
  const auto stop_line_p1 = convertToGeomPoint(stop_line.at(0));
  const auto stop_line_p2 = convertToGeomPoint(stop_line.at(1));

  return findCollisionSegment(path, stop_line_p1, stop_line_p2);
}

template <class T>
std::optional<PathIndexWithOffset> findForwardOffsetSegment(
  const T & path, const size_t base_idx, const double offset_length)
{
  double sum_length = 0.0;
  for (size_t i = base_idx; i < path.points.size() - 1; ++i) {
    sum_length +=
      autoware::universe_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));

    // If it's over offset point, return front index and remain offset length
    if (sum_length >= offset_length) {
      return std::make_pair(i, sum_length - offset_length);
    }
  }

  // No enough path length
  return {};
}

template <class T>
std::optional<PathIndexWithOffset> findBackwardOffsetSegment(
  const T & path, const size_t base_idx, const double offset_length)
{
  double sum_length = 0.0;
  const auto start = static_cast<std::int32_t>(base_idx) - 1;
  for (std::int32_t i = start; i >= 0; --i) {
    sum_length +=
      autoware::universe_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));

    // If it's over offset point, return front index and remain offset length
    if (sum_length >= offset_length) {
      const auto k = static_cast<std::size_t>(i);
      return std::make_pair(k, sum_length - offset_length);
    }
  }

  // No enough path length
  return {};
}

template <class T>
std::optional<PathIndexWithOffset> findOffsetSegment(
  const T & path, const PathIndexWithPoint & collision_segment, const double offset_length)
{
  const size_t collision_idx = collision_segment.first;
  const auto & collision_point = collision_segment.second;

  if (offset_length >= 0) {
    return findForwardOffsetSegment(
      path, collision_idx,
      offset_length +
        autoware::universe_utils::calcDistance2d(path.points.at(collision_idx), collision_point));
  }

  return findBackwardOffsetSegment(
    path, collision_idx + 1,
    -offset_length +
      autoware::universe_utils::calcDistance2d(path.points.at(collision_idx + 1), collision_point));
}

std::optional<PathIndexWithOffset> findOffsetSegment(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const size_t index, const double offset);

template <class T>
geometry_msgs::msg::Pose calcTargetPose(const T & path, const PathIndexWithOffset & offset_segment)
{
  const size_t offset_idx = offset_segment.first;
  const double remain_offset_length = offset_segment.second;
  const auto & p_front = path.points.at(offset_idx).point.pose.position;
  const auto & p_back = path.points.at(offset_idx + 1).point.pose.position;

  // To Eigen point
  const auto p_eigen_front = Eigen::Vector2d(p_front.x, p_front.y);
  const auto p_eigen_back = Eigen::Vector2d(p_back.x, p_back.y);

  // Calculate interpolation ratio
  const auto interpolate_ratio = remain_offset_length / (p_eigen_back - p_eigen_front).norm();

  // Add offset to front point
  const auto target_point_2d = p_eigen_front + interpolate_ratio * (p_eigen_back - p_eigen_front);
  const double interpolated_z = p_front.z + interpolate_ratio * (p_back.z - p_front.z);

  // Calculate orientation so that X-axis would be along the trajectory
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = target_point_2d.x();
  target_pose.position.y = target_point_2d.y();
  target_pose.position.z = interpolated_z;
  const double yaw = autoware::universe_utils::calcAzimuthAngle(p_front, p_back);
  target_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  return target_pose;
}

std::optional<PathIndexWithPose> createTargetPoint(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const LineString2d & stop_line,
  const double margin, const double vehicle_offset);

}  // namespace arc_lane_utils
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__ARC_LANE_UTIL_HPP_
