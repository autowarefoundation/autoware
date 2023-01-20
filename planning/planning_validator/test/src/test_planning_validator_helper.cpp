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

#include "test_planning_validator_helper.hpp"

#include <math.h>

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

Trajectory generateTrajectory(double interval_distance)
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

Trajectory generateNanTrajectory()
{
  Trajectory traj = generateTrajectory(1.0);
  traj.points.front().pose.position.x = NAN;
  return traj;
}

Trajectory generateInfTrajectory()
{
  Trajectory traj = generateTrajectory(1.0);
  traj.points.front().pose.position.x = INFINITY;
  return traj;
}

Trajectory generateBadCurvatureTrajectory()
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

Odometry generateDefaultOdometry(const double x, const double y, const double vx)
{
  Odometry odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.twist.twist.linear.x = vx;
  return odom;
}

rclcpp::NodeOptions getNodeOptionsWithDefaultParams()
{
  rclcpp::NodeOptions node_options;

  // for planning validator
  node_options.append_parameter_override("publish_diag", true);
  node_options.append_parameter_override("invalid_trajectory_handling_type", 0);
  node_options.append_parameter_override("diag_error_count_threshold", 0);
  node_options.append_parameter_override("display_on_terminal", false);
  node_options.append_parameter_override("thresholds.interval", ERROR_INTERVAL);
  node_options.append_parameter_override("thresholds.relative_angle", 1.0);
  node_options.append_parameter_override("thresholds.curvature", ERROR_CURVATURE);
  node_options.append_parameter_override("thresholds.lateral_acc", 100.0);
  node_options.append_parameter_override("thresholds.longitudinal_max_acc", 100.0);
  node_options.append_parameter_override("thresholds.longitudinal_min_acc", -100.0);
  node_options.append_parameter_override("thresholds.steering", 100.0);
  node_options.append_parameter_override("thresholds.steering_rate", 100.0);
  node_options.append_parameter_override("thresholds.velocity_deviation", 100.0);
  node_options.append_parameter_override("thresholds.distance_deviation", 100.0);

  // for vehicle info
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", 3.0);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);

  return node_options;
}
