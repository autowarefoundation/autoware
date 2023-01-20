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

#ifndef TEST_PLANNING_VALIDATOR_HELPER_HPP_
#define TEST_PLANNING_VALIDATOR_HELPER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

constexpr double NOMINAL_INTERVAL = 1.0;
constexpr double ERROR_INTERVAL = 1000.0;
constexpr double ERROR_CURVATURE = 2.0;

using autoware_auto_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

Trajectory generateTrajectory(double interval_distance);

Trajectory generateNanTrajectory();

Trajectory generateInfTrajectory();

Trajectory generateBadCurvatureTrajectory();

Odometry generateDefaultOdometry(const double x = 0.0, const double y = 0.0, const double vx = 0.0);

rclcpp::NodeOptions getNodeOptionsWithDefaultParams();

#endif  // TEST_PLANNING_VALIDATOR_HELPER_HPP_
