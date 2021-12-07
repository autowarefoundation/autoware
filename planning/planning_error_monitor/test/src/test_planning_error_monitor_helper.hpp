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

#ifndef TEST_PLANNING_ERROR_MONITOR_HELPER_HPP_
#define TEST_PLANNING_ERROR_MONITOR_HELPER_HPP_

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

inline Trajectory generateTrajectory(double interval_distance)
{
  Trajectory traj;
  for (double s = 0.0; s <= 10.0 * interval_distance; s += interval_distance) {
    TrajectoryPoint p;
    p.pose.position.x = s;
    p.longitudinal_velocity_mps = 1.0;
    traj.points.push_back(p);
  }
  return traj;
}

inline Trajectory generateNanTrajectory()
{
  Trajectory traj = generateTrajectory(1.0);
  traj.points.front().pose.position.x = NAN;
  return traj;
}

inline Trajectory generateInfTrajectory()
{
  Trajectory traj = generateTrajectory(1.0);
  traj.points.front().pose.position.x = INFINITY;
  return traj;
}

inline Trajectory generateBadCurvatureTrajectory()
{
  Trajectory traj;

  double y = 1.5;
  for (double s = 0.0; s <= 10.0; s += 1.0) {
    TrajectoryPoint p;
    p.longitudinal_velocity_mps = 1.0;
    p.pose.position.x = s;
    p.pose.position.y = y;
    y *= -1.0;  // invert sign
    traj.points.push_back(p);
  }

  return traj;
}

#endif  // TEST_PLANNING_ERROR_MONITOR_HELPER_HPP_
