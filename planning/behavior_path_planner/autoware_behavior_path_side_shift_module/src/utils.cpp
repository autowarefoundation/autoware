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

#include "autoware/behavior_path_side_shift_module/utils.hpp"

#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2/utils.h>

#include <cmath>

namespace autoware::behavior_path_planner
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

bool isAlmostZero(double v)
{
  return std::fabs(v) < 1.0e-4;
}

}  // namespace autoware::behavior_path_planner
