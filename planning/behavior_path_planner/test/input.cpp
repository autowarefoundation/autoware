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
#include "input.hpp"

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathPoint;
using geometry_msgs::msg::Twist;
PathWithLaneId generateStraightSamplePathWithLaneId(
  float initial_pose_value, float pose_increment, size_t point_sample)
{
  PathWithLaneId path;
  for (size_t idx = 0; idx < point_sample; ++idx) {
    PathPoint point;
    point.pose.position.x = std::exchange(initial_pose_value, initial_pose_value + pose_increment);
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0;
    point.longitudinal_velocity_mps = 0.1;  // [m/s]
    point.heading_rate_rps = 0.0;           // [rad/s]
    point.is_final = (idx == point_sample - 1);

    PathPointWithLaneId path_point_with_lane_id;
    path_point_with_lane_id.point = point;
    path_point_with_lane_id.lane_ids = std::vector<int64>();

    path.header.frame_id = "map";
    path.points.push_back(path_point_with_lane_id);
  }

  return path;
}

PathWithLaneId generateDiagonalSamplePathWithLaneId(
  float initial_pose_value, float pose_increment, size_t point_sample)
{
  PathWithLaneId path;
  for (size_t idx = 0; idx < point_sample; ++idx) {
    PathPoint point;
    point.pose.position.x = std::exchange(initial_pose_value, initial_pose_value + pose_increment);
    point.pose.position.y = point.pose.position.x;
    point.pose.position.z = 0.0;
    point.longitudinal_velocity_mps = 0.1;  // [m/s]
    point.heading_rate_rps = 0.0;           // [rad/s]
    point.is_final = (idx == point_sample - 1);

    PathPointWithLaneId path_point_with_lane_id;
    path_point_with_lane_id.point = point;
    path_point_with_lane_id.lane_ids = std::vector<int64>();

    path.header.frame_id = "map";
    path.points.push_back(path_point_with_lane_id);
  }

  return path;
}

Twist generateSampleEgoTwist(float && l_x, float && l_y, float && l_z)
{
  Twist twist;
  twist.linear.x = l_x;
  twist.linear.y = l_y;
  twist.linear.z = l_z;

  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  return twist;
}

Pose generateEgoSamplePose(float && p_x, float && p_y, float && p_z)
{
  Pose pose;
  pose.position.x = p_x;
  pose.position.y = p_y;
  pose.position.z = p_z;
  return pose;
}
}  // namespace behavior_path_planner
