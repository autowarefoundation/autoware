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

#ifndef UTILIZATION__ARC_LANE_UTIL_HPP_
#define UTILIZATION__ARC_LANE_UTIL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <boost/optional.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace
{
geometry_msgs::msg::Point convertToGeomPoint(const Point2d & p)
{
  geometry_msgs::msg::Point geom_p;
  geom_p.x = p.x();
  geom_p.y = p.y();

  return geom_p;
}

geometry_msgs::msg::Point operator+(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  geometry_msgs::msg::Point p;
  p.x = p1.x + p2.x;
  p.y = p1.y + p2.y;
  p.z = p1.z + p2.z;

  return p;
}

[[maybe_unused]] geometry_msgs::msg::Point operator*(
  const geometry_msgs::msg::Point & p, const double v)
{
  geometry_msgs::msg::Point multiplied_p;
  multiplied_p.x = p.x * v;
  multiplied_p.y = p.y * v;
  multiplied_p.z = p.z * v;

  return multiplied_p;
}

[[maybe_unused]] geometry_msgs::msg::Point operator*(
  const double v, const geometry_msgs::msg::Point & p)
{
  return p * v;
}
}  // namespace

namespace arc_lane_utils
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;    // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, Point2d>;                  // front index, point2d
using PathIndexWithPoint = std::pair<size_t, geometry_msgs::msg::Point>;  // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                    // front index, offset

inline double calcSignedDistance(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Point & p2)
{
  Eigen::Affine3d map2p1;
  tf2::fromMsg(p1, map2p1);
  const auto basecoords_p2 = map2p1.inverse() * Eigen::Vector3d(p2.x, p2.y, p2.z);
  return basecoords_p2.x() >= 0 ? basecoords_p2.norm() : -basecoords_p2.norm();
}

// calculate one collision point between the line (from p1 to p2) and the line (from p3 to p4)
inline boost::optional<geometry_msgs::msg::Point> checkCollision(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
{
  const double det = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);

  if (det == 0.0) {
    // collision is not one point.
    return boost::none;
  }

  const double t1 = ((p4.y - p3.y) * (p4.x - p1.x) - (p4.x - p3.x) * (p4.y - p1.y)) / det;
  const double t2 = ((p2.x - p1.x) * (p4.y - p1.y) - (p2.y - p1.y) * (p4.x - p1.x)) / det;

  // check collision is outside the segment line
  if (t1 < 0.0 || 1.0 < t1 || t2 < 0.0 || 1.0 < t2) {
    return boost::none;
  }

  return p1 * (1.0 - t1) + p2 * t1;
}

template <class T>
boost::optional<PathIndexWithPoint> findCollisionSegment(
  const T & path, const geometry_msgs::msg::Point & stop_line_p1,
  const geometry_msgs::msg::Point & stop_line_p2, const size_t target_lane_id)
{
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto & prev_lane_ids = path.points.at(i).lane_ids;
    const auto & next_lane_ids = path.points.at(i + 1).lane_ids;

    const bool is_target_lane_in_prev_lane =
      std::find(prev_lane_ids.begin(), prev_lane_ids.end(), target_lane_id) != prev_lane_ids.end();
    const bool is_target_lane_in_next_lane =
      std::find(next_lane_ids.begin(), next_lane_ids.end(), target_lane_id) != next_lane_ids.end();
    if (!is_target_lane_in_prev_lane && !is_target_lane_in_next_lane) {
      continue;
    }

    const auto & p1 =
      tier4_autoware_utils::getPoint(path.points.at(i));  // Point before collision point
    const auto & p2 =
      tier4_autoware_utils::getPoint(path.points.at(i + 1));  // Point after collision point

    const auto collision_point = checkCollision(p1, p2, stop_line_p1, stop_line_p2);

    if (collision_point) {
      return std::make_pair(i, collision_point.get());
    }
  }

  return {};
}

template <class T>
boost::optional<PathIndexWithPoint> findCollisionSegment(
  const T & path, const LineString2d & stop_line, const size_t target_lane_id)
{
  const auto stop_line_p1 = convertToGeomPoint(stop_line.at(0));
  const auto stop_line_p2 = convertToGeomPoint(stop_line.at(1));

  return findCollisionSegment(path, stop_line_p1, stop_line_p2, target_lane_id);
}

template <class T>
boost::optional<PathIndexWithOffset> findForwardOffsetSegment(
  const T & path, const size_t base_idx, const double offset_length)
{
  double sum_length = 0.0;
  for (size_t i = base_idx; i < path.points.size() - 1; ++i) {
    sum_length += tier4_autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));

    // If it's over offset point, return front index and remain offset length
    if (sum_length >= offset_length) {
      return std::make_pair(i, sum_length - offset_length);
    }
  }

  // No enough path length
  return {};
}

template <class T>
boost::optional<PathIndexWithOffset> findBackwardOffsetSegment(
  const T & path, const size_t base_idx, const double offset_length)
{
  double sum_length = 0.0;
  const auto start = static_cast<std::int32_t>(base_idx) - 1;
  for (std::int32_t i = start; i >= 0; --i) {
    sum_length += tier4_autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));

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
boost::optional<PathIndexWithOffset> findOffsetSegment(
  const T & path, const PathIndexWithPoint & collision_segment, const double offset_length)
{
  const size_t collision_idx = collision_segment.first;
  const auto & collision_point = collision_segment.second;

  if (offset_length >= 0) {
    return findForwardOffsetSegment(
      path, collision_idx,
      offset_length +
        tier4_autoware_utils::calcDistance2d(path.points.at(collision_idx), collision_point));
  }

  return findBackwardOffsetSegment(
    path, collision_idx + 1,
    -offset_length +
      tier4_autoware_utils::calcDistance2d(path.points.at(collision_idx + 1), collision_point));
}

inline boost::optional<PathIndexWithOffset> findOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t index,
  const double offset)
{
  if (offset >= 0) {
    return findForwardOffsetSegment(path, index, offset);
  }

  return findBackwardOffsetSegment(path, index, -offset);
}

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
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(p_front, p_back);
  target_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);
  return target_pose;
}

inline boost::optional<PathIndexWithPose> createTargetPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const LineString2d & stop_line,
  const size_t lane_id, const double margin, const double vehicle_offset)
{
  // Find collision segment
  const auto collision_segment = findCollisionSegment(path, stop_line, lane_id);
  if (!collision_segment) {
    // No collision
    return {};
  }

  // Calculate offset length from stop line
  // Use '-' to make the positive direction is forward
  const double offset_length = -(margin + vehicle_offset);

  // Find offset segment
  const auto offset_segment = findOffsetSegment(path, *collision_segment, offset_length);
  if (!offset_segment) {
    // No enough path length
    return {};
  }

  const auto target_pose = calcTargetPose(path, *offset_segment);

  const auto front_idx = offset_segment->first;
  return std::make_pair(front_idx, target_pose);
}
}  // namespace arc_lane_utils
}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__ARC_LANE_UTIL_HPP_
