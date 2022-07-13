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

#ifndef VEHICLE__TEST_VEHICLE_STATE_CHECKER_HELPER_HPP_
#define VEHICLE__TEST_VEHICLE_STATE_CHECKER_HELPER_HPP_

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

inline Trajectory generateTrajectoryWithStopPoint(const geometry_msgs::msg::Pose & goal_pose)
{
  constexpr double interval_distance = 1.0;

  Trajectory traj;
  for (double s = 0.0; s <= 10.0 * interval_distance; s += interval_distance) {
    TrajectoryPoint p;
    p.pose = goal_pose;
    p.pose.position.x += s;
    p.longitudinal_velocity_mps = 1.0;
    traj.points.push_back(p);
  }

  traj.points.front().longitudinal_velocity_mps = 0.0;
  std::reverse(traj.points.begin(), traj.points.end());
  return traj;
}

inline Trajectory generateTrajectoryWithoutStopPoint(const geometry_msgs::msg::Pose & goal_pose)
{
  constexpr double interval_distance = 1.0;

  Trajectory traj;
  for (double s = 0.0; s <= 10.0 * interval_distance; s += interval_distance) {
    TrajectoryPoint p;
    p.pose = goal_pose;
    p.pose.position.x += s;
    p.longitudinal_velocity_mps = 1.0;
    traj.points.push_back(p);
  }

  std::reverse(traj.points.begin(), traj.points.end());
  return traj;
}

#endif  // VEHICLE__TEST_VEHICLE_STATE_CHECKER_HELPER_HPP_
