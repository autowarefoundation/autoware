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

#ifndef INPUT_HPP_
#define INPUT_HPP_

#endif  // INPUT_HPP_

#include "autoware_planning_msgs/msg/path_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tier4_planning_msgs/msg/path_point_with_lane_id.hpp"
#include "tier4_planning_msgs/msg/path_with_lane_id.hpp"

#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_planning_msgs::msg::PathPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;
PathWithLaneId generateStraightSamplePathWithLaneId(
  float initial_pose_value, float pose_increment, size_t point_sample);

PathWithLaneId generateDiagonalSamplePathWithLaneId(
  float initial_pose_value, float pose_increment, size_t point_sample);

Twist generateSampleEgoTwist(float && l_x, float && l_y, float && l_z);

Pose generateEgoSamplePose(float && p_x, float && p_y, float && p_z);
}  // namespace autoware::behavior_path_planner
