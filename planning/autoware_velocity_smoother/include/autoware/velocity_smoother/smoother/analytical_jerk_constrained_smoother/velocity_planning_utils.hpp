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

#ifndef AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__ANALYTICAL_JERK_CONSTRAINED_SMOOTHER__VELOCITY_PLANNING_UTILS_HPP_  // NOLINT
#define AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__ANALYTICAL_JERK_CONSTRAINED_SMOOTHER__VELOCITY_PLANNING_UTILS_HPP_  // NOLINT

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace autoware::velocity_smoother
{
namespace analytical_velocity_planning_utils
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<autoware_planning_msgs::msg::TrajectoryPoint>;

bool calcStopDistWithJerkAndAccConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, int & type, std::vector<double> & times,
  double & stop_dist);
bool validCheckCalcStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);
bool calcStopVelocityWithConstantJerkAccLimit(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double decel_target_vel, const int type,
  const std::vector<double> & times, const size_t start_index,
  TrajectoryPoints & output_trajectory);
void updateStopVelocityStatus(
  double v0, double a0, double jerk_acc, double jerk_dec, int type,
  const std::vector<double> & times, double t, double & x, double & v, double & a, double & j);
double integ_x(double x0, double v0, double a0, double j0, double t);
double integ_v(double v0, double a0, double j0, double t);
double integ_a(double a0, double j0, double t);
}  // namespace analytical_velocity_planning_utils
}  // namespace autoware::velocity_smoother

// clang-format off
#endif  // AUTOWARE__VELOCITY_SMOOTHER__SMOOTHER__ANALYTICAL_JERK_CONSTRAINED_SMOOTHER__VELOCITY_PLANNING_UTILS_HPP_  // NOLINT
// clang-format on
