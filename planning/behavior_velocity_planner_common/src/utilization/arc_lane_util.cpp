// Copyright 2023 TIER IV, Inc.
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

#include <behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

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

namespace
{
geometry_msgs::msg::Point operator+(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  geometry_msgs::msg::Point p;
  p.x = p1.x + p2.x;
  p.y = p1.y + p2.y;
  p.z = p1.z + p2.z;

  return p;
}

geometry_msgs::msg::Point operator*(const geometry_msgs::msg::Point & p, const double v)
{
  geometry_msgs::msg::Point multiplied_p;
  multiplied_p.x = p.x * v;
  multiplied_p.y = p.y * v;
  multiplied_p.z = p.z * v;

  return multiplied_p;
}

/*
geometry_msgs::msg::Point operator*(const double v, const geometry_msgs::msg::Point & p)
{
return p * v;
}
*/
}  // namespace

namespace behavior_velocity_planner::arc_lane_utils
{

double calcSignedDistance(const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Point & p2)
{
  Eigen::Affine3d map2p1;
  tf2::fromMsg(p1, map2p1);
  const auto basecoords_p2 = map2p1.inverse() * Eigen::Vector3d(p2.x, p2.y, p2.z);
  return basecoords_p2.x() >= 0 ? basecoords_p2.norm() : -basecoords_p2.norm();
}

// calculate one collision point between the line (from p1 to p2) and the line (from p3 to p4)

std::optional<geometry_msgs::msg::Point> checkCollision(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4)
{
  const double det = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);

  if (det == 0.0) {
    // collision is not one point.
    return std::nullopt;
  }

  const double t1 = ((p4.y - p3.y) * (p4.x - p1.x) - (p4.x - p3.x) * (p4.y - p1.y)) / det;
  const double t2 = ((p2.x - p1.x) * (p4.y - p1.y) - (p2.y - p1.y) * (p4.x - p1.x)) / det;

  // check collision is outside the segment line
  if (t1 < 0.0 || 1.0 < t1 || t2 < 0.0 || 1.0 < t2) {
    return std::nullopt;
  }

  return p1 * (1.0 - t1) + p2 * t1;
}

std::optional<PathIndexWithOffset> findOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t index,
  const double offset)
{
  if (offset >= 0) {
    return findForwardOffsetSegment(path, index, offset);
  }

  return findBackwardOffsetSegment(path, index, -offset);
}

std::optional<PathIndexWithPose> createTargetPoint(
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
}  // namespace behavior_velocity_planner::arc_lane_utils
