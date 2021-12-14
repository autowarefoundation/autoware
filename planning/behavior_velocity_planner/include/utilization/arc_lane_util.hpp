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

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
namespace arc_lane_utils
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;  // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, Point2d>;                // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                  // front index, offset

inline double calcSignedDistance(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Point & p2)
{
  Eigen::Affine3d map2p1;
  tf2::fromMsg(p1, map2p1);
  const auto basecoords_p2 = map2p1.inverse() * Eigen::Vector3d(p2.x, p2.y, p2.z);
  return basecoords_p2.x() >= 0 ? basecoords_p2.norm() : -basecoords_p2.norm();
}

inline boost::optional<Point2d> getNearestCollisionPoint(
  const LineString2d & stop_line, const LineString2d & path_segment)
{
  // Find all collision points
  std::vector<Point2d> collision_points;
  bg::intersection(stop_line, path_segment, collision_points);
  if (collision_points.empty()) {
    return {};
  }

  // To dist list
  std::vector<double> dist_list;
  dist_list.reserve(collision_points.size());
  std::transform(
    collision_points.cbegin(), collision_points.cend(), std::back_inserter(dist_list),
    [&path_segment](const Point2d & collision_point) {
      return bg::distance(path_segment.front(), collision_point);
    });

  // Find nearest collision point
  const auto min_itr = std::min_element(dist_list.cbegin(), dist_list.cend());
  const auto min_idx = std::distance(dist_list.cbegin(), min_itr);

  return collision_points.at(min_idx);
}

template <class T>
boost::optional<PathIndexWithPoint2d> findCollisionSegment(
  const T & path, const LineString2d & stop_line)
{
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto & p1 = path.points.at(i).point.pose.position;      // Point before collision point
    const auto & p2 = path.points.at(i + 1).point.pose.position;  // Point after collision point

    const LineString2d path_segment = {{p1.x, p1.y}, {p2.x, p2.y}};

    const auto nearest_collision_point = getNearestCollisionPoint(stop_line, path_segment);
    if (nearest_collision_point) {
      return std::make_pair(i, *nearest_collision_point);
    }
  }

  return {};
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
  const T & path, const PathIndexWithPoint2d & collision_segment, const double offset_length)
{
  const size_t collision_idx = collision_segment.first;
  const Point2d & collision_point = collision_segment.second;
  const auto p_front = to_bg2d(path.points.at(collision_idx).point.pose.position);
  const auto p_back = to_bg2d(path.points.at(collision_idx + 1).point.pose.position);

  if (offset_length >= 0) {
    return findForwardOffsetSegment(
      path, collision_idx, offset_length + bg::distance(p_front, collision_point));
  } else {
    return findBackwardOffsetSegment(
      path, collision_idx + 1, -offset_length + bg::distance(p_back, collision_point));
  }
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

}  // namespace arc_lane_utils
}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__ARC_LANE_UTIL_HPP_
