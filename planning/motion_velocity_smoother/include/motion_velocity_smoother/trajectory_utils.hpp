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

#ifndef MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_
#define MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_

#include "autoware_auto_planning_msgs/msg/detail/trajectory_point__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#include <map>
#include <optional>
#include <tuple>
#include <vector>

namespace motion_velocity_smoother::trajectory_utils
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using geometry_msgs::msg::Pose;

TrajectoryPoint calcInterpolatedTrajectoryPoint(
  const TrajectoryPoints & trajectory, const Pose & target_pose, const size_t seg_idx);

TrajectoryPoints extractPathAroundIndex(
  const TrajectoryPoints & trajectory, const size_t index, const double & ahead_length,
  const double & behind_length);

std::vector<double> calcArclengthArray(const TrajectoryPoints & trajectory);

std::vector<double> calcTrajectoryIntervalDistance(const TrajectoryPoints & trajectory);

std::vector<double> calcTrajectoryCurvatureFrom3Points(
  const TrajectoryPoints & trajectory, size_t idx_dist);

void setZeroVelocity(TrajectoryPoints & trajectory);

double getMaxVelocity(const TrajectoryPoints & trajectory);

double getMaxAbsVelocity(const TrajectoryPoints & trajectory);

void applyMaximumVelocityLimit(
  const size_t from, const size_t to, const double max_vel, TrajectoryPoints & trajectory);

std::optional<size_t> searchZeroVelocityIdx(const TrajectoryPoints & trajectory);

bool calcStopDistWithJerkConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, std::map<double, double> & jerk_profile,
  double & stop_dist);

bool isValidStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin);

std::optional<TrajectoryPoints> applyDecelFilterWithJerkConstraint(
  const TrajectoryPoints & input, const size_t start_index, const double v0, const double a0,
  const double min_acc, const double decel_target_vel,
  const std::map<double, double> & jerk_profile);

std::optional<std::tuple<double, double, double, double>> updateStateWithJerkConstraint(
  const double v0, const double a0, const std::map<double, double> & jerk_profile, const double t);

std::vector<double> calcVelocityProfileWithConstantJerkAndAccelerationLimit(
  const TrajectoryPoints & trajectory, const double v0, const double a0, const double jerk,
  const double acc_max, const double acc_min);

double calcStopDistance(const TrajectoryPoints & trajectory, const size_t closest);

}  // namespace motion_velocity_smoother::trajectory_utils

#endif  // MOTION_VELOCITY_SMOOTHER__TRAJECTORY_UTILS_HPP_
