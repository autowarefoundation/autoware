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

#ifndef BEHAVIOR_PATH_SIDE_SHIFT_MODULE__UTILS_HPP_
#define BEHAVIOR_PATH_SIDE_SHIFT_MODULE__UTILS_HPP_

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;

void setOrientation(PathWithLaneId * path);

bool getStartAvoidPose(
  const PathWithLaneId & path, const double start_distance, const size_t nearest_idx,
  Pose * start_avoid_pose);

bool isAlmostZero(double v);

Point transformToGrid(
  const Point & pt, const double longitudinal_offset, const double lateral_offset, const double yaw,
  const TransformStamped & geom_tf);

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_SIDE_SHIFT_MODULE__UTILS_HPP_
