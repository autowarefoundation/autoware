// Copyright 2021 TIER IV, Inc.
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

#include "behavior_path_side_shift_module/utils.hpp"

#include "behavior_path_planner_common/utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.h>

#include <cmath>

namespace behavior_path_planner
{
void setOrientation(PathWithLaneId * path)
{
  if (!path) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("side_shift").get_child("util"),
      "Pointer to path is NULL!");
  }

  // Reset orientation
  for (size_t idx = 0; idx < path->points.size(); ++idx) {
    double angle = 0.0;
    auto & pt = path->points.at(idx);
    if (idx + 1 < path->points.size()) {
      const auto next_pt = path->points.at(idx + 1);
      angle = std::atan2(
        next_pt.point.pose.position.y - pt.point.pose.position.y,
        next_pt.point.pose.position.x - pt.point.pose.position.x);
    } else if (idx != 0) {
      const auto prev_pt = path->points.at(idx - 1);
      angle = std::atan2(
        pt.point.pose.position.y - prev_pt.point.pose.position.y,
        pt.point.pose.position.x - prev_pt.point.pose.position.x);
    }
    tf2::Quaternion yaw_quat;
    yaw_quat.setRPY(0, 0, angle);
    pt.point.pose.orientation = tf2::toMsg(yaw_quat);
  }
}

bool getStartAvoidPose(
  const PathWithLaneId & path, const double start_distance, const size_t nearest_idx,
  Pose * start_avoid_pose)
{
  if (!start_avoid_pose) {
    return false;
  }
  if (nearest_idx >= path.points.size()) {
    return false;
  }

  double arclength = 0.0;
  for (size_t idx = nearest_idx + 1; idx < path.points.size(); ++idx) {
    const auto pt = path.points.at(idx).point;
    const auto pt_prev = path.points.at(idx - 1).point;
    const double dx = pt.pose.position.x - pt_prev.pose.position.x;
    const double dy = pt.pose.position.y - pt_prev.pose.position.y;
    arclength += std::hypot(dx, dy);

    if (arclength > start_distance) {
      *start_avoid_pose = pt.pose;
      return true;
    }
  }

  return false;
}

bool isAlmostZero(double v)
{
  return std::fabs(v) < 1.0e-4;
}

Point transformToGrid(
  const Point & pt, const double longitudinal_offset, const double lateral_offset, const double yaw,
  const TransformStamped & geom_tf)
{
  Point offset_pt, grid_pt;
  offset_pt = pt;
  offset_pt.x += longitudinal_offset * cos(yaw) - lateral_offset * sin(yaw);
  offset_pt.y += longitudinal_offset * sin(yaw) + lateral_offset * cos(yaw);
  tf2::doTransform(offset_pt, grid_pt, geom_tf);
  return grid_pt;
}

}  // namespace behavior_path_planner
