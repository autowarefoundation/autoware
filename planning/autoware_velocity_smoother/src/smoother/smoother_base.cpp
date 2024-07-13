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

#include "autoware/velocity_smoother/smoother/smoother_base.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"
#include "autoware/velocity_smoother/resample.hpp"
#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::velocity_smoother
{

namespace
{
TrajectoryPoints applyPreProcess(
  const TrajectoryPoints & input, const double interval, const bool use_resampling)
{
  using autoware::motion_utils::calcArcLength;
  using autoware::motion_utils::convertToTrajectory;
  using autoware::motion_utils::convertToTrajectoryPointArray;
  using autoware::motion_utils::resampleTrajectory;

  if (!use_resampling) {
    return input;
  }

  TrajectoryPoints output;
  std::vector<double> arc_length;

  // since the resampling takes a long time, omit the resampling when it is not requested
  const auto traj_length = calcArcLength(input);
  for (double s = 0; s < traj_length; s += interval) {
    arc_length.push_back(s);
  }

  const auto points = resampleTrajectory(convertToTrajectory(input), arc_length);
  output = convertToTrajectoryPointArray(points);
  output.back() = input.back();  // keep the final speed.

  return output;
}
}  // namespace

SmootherBase::SmootherBase(
  rclcpp::Node & node, const std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper)
: time_keeper_(time_keeper)
{
  auto & p = base_param_;
  p.max_accel = node.declare_parameter<double>("normal.max_acc");
  p.min_decel = node.declare_parameter<double>("normal.min_acc");
  p.stop_decel = node.declare_parameter<double>("stop_decel");
  p.max_jerk = node.declare_parameter<double>("normal.max_jerk");
  p.min_jerk = node.declare_parameter<double>("normal.min_jerk");
  p.max_lateral_accel = node.declare_parameter<double>("max_lateral_accel");
  p.min_decel_for_lateral_acc_lim_filter =
    node.declare_parameter<double>("min_decel_for_lateral_acc_lim_filter");
  p.sample_ds = node.declare_parameter<double>("resample_ds");
  p.curvature_threshold = node.declare_parameter<double>("curvature_threshold");
  p.max_steering_angle_rate = node.declare_parameter<double>("max_steering_angle_rate");
  p.curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");
  p.decel_distance_before_curve = node.declare_parameter<double>("decel_distance_before_curve");
  p.decel_distance_after_curve = node.declare_parameter<double>("decel_distance_after_curve");
  p.min_curve_velocity = node.declare_parameter<double>("min_curve_velocity");
  p.resample_param.max_trajectory_length = node.declare_parameter<double>("max_trajectory_length");
  p.resample_param.min_trajectory_length = node.declare_parameter<double>("min_trajectory_length");
  p.resample_param.resample_time = node.declare_parameter<double>("resample_time");
  p.resample_param.dense_resample_dt = node.declare_parameter<double>("dense_resample_dt");
  p.resample_param.dense_min_interval_distance =
    node.declare_parameter<double>("dense_min_interval_distance");
  p.resample_param.sparse_resample_dt = node.declare_parameter<double>("sparse_resample_dt");
  p.resample_param.sparse_min_interval_distance =
    node.declare_parameter<double>("sparse_min_interval_distance");
}

void SmootherBase::setWheelBase(const double wheel_base)
{
  base_param_.wheel_base = wheel_base;
}

void SmootherBase::setParam(const BaseParam & param)
{
  base_param_ = param;
}

SmootherBase::BaseParam SmootherBase::getBaseParam() const
{
  return base_param_;
}

double SmootherBase::getMaxAccel() const
{
  return base_param_.max_accel;
}

double SmootherBase::getMinDecel() const
{
  return base_param_.min_decel;
}

double SmootherBase::getMaxJerk() const
{
  return base_param_.max_jerk;
}

double SmootherBase::getMinJerk() const
{
  return base_param_.min_jerk;
}

TrajectoryPoints SmootherBase::applyLateralAccelerationFilter(
  const TrajectoryPoints & input, [[maybe_unused]] const double v0,
  [[maybe_unused]] const double a0, [[maybe_unused]] const bool enable_smooth_limit,
  const bool use_resampling, const double input_points_interval) const
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (input.size() < 3) {
    return input;  // cannot calculate lateral acc. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.
  TrajectoryPoints output;
  const double points_interval =
    use_resampling ? base_param_.sample_ds : input_points_interval;  // [m]
  // since the resampling takes a long time, omit the resampling when it is not requested
  if (use_resampling) {
    std::vector<double> out_arclength;
    const auto traj_length = autoware::motion_utils::calcArcLength(input);
    for (double s = 0; s < traj_length; s += points_interval) {
      out_arclength.push_back(s);
    }
    const auto output_traj = autoware::motion_utils::resampleTrajectory(
      autoware::motion_utils::convertToTrajectory(input), out_arclength);
    output = autoware::motion_utils::convertToTrajectoryPointArray(output_traj);
    output.back() = input.back();  // keep the final speed.
  } else {
    output = input;
  }

  const size_t idx_dist = static_cast<size_t>(
    std::max(static_cast<int>((base_param_.curvature_calculation_distance) / points_interval), 1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  //  Decrease speed according to lateral G
  const size_t before_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_before_curve / points_interval));
  const size_t after_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_after_curve / points_interval));
  const double max_lateral_accel_abs = std::fabs(base_param_.max_lateral_accel);

  const auto latacc_min_vel_arr =
    enable_smooth_limit ? trajectory_utils::calcVelocityProfileWithConstantJerkAndAccelerationLimit(
                            output, v0, a0, base_param_.min_jerk, base_param_.max_accel,
                            base_param_.min_decel_for_lateral_acc_lim_filter)
                        : std::vector<double>{};

  for (size_t i = 0; i < output.size(); ++i) {
    double curvature = 0.0;
    const size_t start = i > after_decel_index ? i - after_decel_index : 0;
    const size_t end = std::min(output.size(), i + before_decel_index + 1);
    for (size_t j = start; j < end; ++j) {
      if (j >= curvature_v.size()) return output;
      curvature = std::max(curvature, std::fabs(curvature_v.at(j)));
    }
    double v_curvature_max = std::sqrt(max_lateral_accel_abs / std::max(curvature, 1.0E-5));
    v_curvature_max = std::max(v_curvature_max, base_param_.min_curve_velocity);

    if (enable_smooth_limit) {
      if (i >= latacc_min_vel_arr.size()) return output;
      v_curvature_max = std::max(v_curvature_max, latacc_min_vel_arr.at(i));
    }
    if (output.at(i).longitudinal_velocity_mps > v_curvature_max) {
      output.at(i).longitudinal_velocity_mps = v_curvature_max;
    }
  }
  return output;
}

TrajectoryPoints SmootherBase::applySteeringRateLimit(
  const TrajectoryPoints & input, const bool use_resampling,
  const double input_points_interval) const
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (input.size() < 3) {
    return input;  // cannot calculate the desired velocity. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.
  const double points_interval = use_resampling ? base_param_.sample_ds : input_points_interval;

  auto output = applyPreProcess(input, points_interval, use_resampling);

  const size_t idx_dist = static_cast<size_t>(
    std::max(static_cast<int>((base_param_.curvature_calculation_distance) / points_interval), 1));

  // Step1. Calculate curvature assuming the trajectory points interval is constant.
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  // Step2. Calculate steer rate for each trajectory point.
  std::vector<double> steer_rate_arr(output.size(), 0.0);
  for (size_t i = 0; i < output.size() - 1; i++) {
    // velocity
    const auto & v_front = output.at(i + 1).longitudinal_velocity_mps;
    const auto & v_back = output.at(i).longitudinal_velocity_mps;
    // steer
    auto & steer_front = output.at(i + 1).front_wheel_angle_rad;
    auto & steer_back = output.at(i).front_wheel_angle_rad;

    // calculate the just 2 steering angle
    steer_front = std::atan(base_param_.wheel_base * curvature_v.at(i + 1));
    steer_back = std::atan(base_param_.wheel_base * curvature_v.at(i));

    const auto mean_vel = 0.5 * (v_front + v_back);
    const auto dt = std::max(points_interval / mean_vel, std::numeric_limits<double>::epsilon());
    const auto steering_diff = std::fabs(steer_front - steer_back);

    steer_rate_arr.at(i) = steering_diff / dt;
  }

  steer_rate_arr.back() = steer_rate_arr.at((output.size() - 2));

  // Step3. Remove noise by mean filter.
  for (size_t i = 1; i < steer_rate_arr.size() - 1; i++) {
    steer_rate_arr.at(i) =
      (steer_rate_arr.at(i - 1) + steer_rate_arr.at(i) + steer_rate_arr.at(i + 1)) / 3.0;
  }

  // Step4. Limit velocity by steer rate.
  for (size_t i = 0; i < output.size() - 1; i++) {
    if (fabs(curvature_v.at(i)) < base_param_.curvature_threshold) {
      continue;
    }

    const auto steer_rate = steer_rate_arr.at(i);
    if (steer_rate < autoware::universe_utils::deg2rad(base_param_.max_steering_angle_rate)) {
      continue;
    }

    const auto mean_vel =
      (output.at(i).longitudinal_velocity_mps + output.at(i + 1).longitudinal_velocity_mps) / 2.0;
    const auto target_mean_vel =
      mean_vel *
      (autoware::universe_utils::deg2rad(base_param_.max_steering_angle_rate) / steer_rate);

    for (size_t k = 0; k < 2; k++) {
      auto & velocity = output.at(i + k).longitudinal_velocity_mps;
      const float target_velocity = std::max(
        base_param_.min_curve_velocity,
        std::min(target_mean_vel, velocity * (target_mean_vel / mean_vel)));
      velocity = std::min(velocity, target_velocity);
    }
  }

  return output;
}

}  // namespace autoware::velocity_smoother
