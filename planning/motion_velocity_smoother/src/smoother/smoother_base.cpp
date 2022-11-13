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

#include "motion_velocity_smoother/smoother/smoother_base.hpp"

#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/tmp_conversion.hpp"
#include "motion_velocity_smoother/resample.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace motion_velocity_smoother
{
using vehicle_info_util::VehicleInfoUtil;

SmootherBase::SmootherBase(rclcpp::Node & node)
{
  auto & p = base_param_;
  p.max_accel = node.declare_parameter("normal.max_acc", 2.0);
  p.min_decel = node.declare_parameter("normal.min_acc", -3.0);
  p.stop_decel = node.declare_parameter("stop_decel", 0.0);
  p.max_jerk = node.declare_parameter("normal.max_jerk", 0.3);
  p.min_jerk = node.declare_parameter("normal.min_jerk", -0.1);
  p.max_lateral_accel = node.declare_parameter("max_lateral_accel", 0.2);
  p.min_decel_for_lateral_acc_lim_filter =
    node.declare_parameter("min_decel_for_lateral_acc_lim_filter", -2.5);
  p.sample_ds = node.declare_parameter("resample_ds", 0.5);
  p.curvature_threshold = node.declare_parameter("curvature_threshold", 0.2);
  p.max_steering_angle_rate = node.declare_parameter("max_steering_angle_rate", 5.0);
  p.curvature_calculation_distance = node.declare_parameter("curvature_calculation_distance", 1.0);
  p.decel_distance_before_curve = node.declare_parameter("decel_distance_before_curve", 3.5);
  p.decel_distance_after_curve = node.declare_parameter("decel_distance_after_curve", 0.0);
  p.min_curve_velocity = node.declare_parameter("min_curve_velocity", 1.38);
  p.resample_param.max_trajectory_length = node.declare_parameter("max_trajectory_length", 200.0);
  p.resample_param.min_trajectory_length = node.declare_parameter("min_trajectory_length", 30.0);
  p.resample_param.resample_time = node.declare_parameter("resample_time", 10.0);
  p.resample_param.dense_resample_dt = node.declare_parameter("dense_resample_dt", 0.1);
  p.resample_param.dense_min_interval_distance =
    node.declare_parameter("dense_min_interval_distance", 0.1);
  p.resample_param.sparse_resample_dt = node.declare_parameter("sparse_resample_dt", 0.5);
  p.resample_param.sparse_min_interval_distance =
    node.declare_parameter("sparse_min_interval_distance", 4.0);
}

void SmootherBase::setParam(const BaseParam & param) { base_param_ = param; }

SmootherBase::BaseParam SmootherBase::getBaseParam() const { return base_param_; }

double SmootherBase::getMaxAccel() const { return base_param_.max_accel; }

double SmootherBase::getMinDecel() const { return base_param_.min_decel; }

double SmootherBase::getMaxJerk() const { return base_param_.max_jerk; }

double SmootherBase::getMinJerk() const { return base_param_.min_jerk; }

boost::optional<TrajectoryPoints> SmootherBase::applyLateralAccelerationFilter(
  const TrajectoryPoints & input, [[maybe_unused]] const double v0,
  [[maybe_unused]] const double a0, [[maybe_unused]] const bool enable_smooth_limit) const
{
  if (input.empty()) {
    return boost::none;
  }

  if (input.size() < 3) {
    return boost::optional<TrajectoryPoints>(input);  // cannot calculate lateral acc. do nothing.
  }

  // Interpolate with constant interval distance for lateral acceleration calculation.
  constexpr double points_interval = 0.1;  // [m]
  std::vector<double> out_arclength;
  const auto traj_length = motion_utils::calcArcLength(input);
  for (double s = 0; s < traj_length; s += points_interval) {
    out_arclength.push_back(s);
  }
  const auto output_traj =
    motion_utils::resampleTrajectory(motion_utils::convertToTrajectory(input), out_arclength);
  auto output = motion_utils::convertToTrajectoryPointArray(output_traj);
  output.back() = input.back();  // keep the final speed.

  constexpr double curvature_calc_dist = 5.0;  // [m] calc curvature with 5m away points
  const size_t idx_dist =
    static_cast<size_t>(std::max(static_cast<int>((curvature_calc_dist) / points_interval), 1));

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

boost::optional<TrajectoryPoints> SmootherBase::applySteeringRateLimit(
  const TrajectoryPoints & input) const
{
  if (input.empty()) {
    return boost::none;
  }

  if (input.size() < 3) {
    return boost::optional<TrajectoryPoints>(
      input);  // cannot calculate the desired velocity. do nothing.
  }
  // Interpolate with constant interval distance for lateral acceleration calculation.
  std::vector<double> out_arclength;
  const auto traj_length = motion_utils::calcArcLength(input);
  for (double s = 0; s < traj_length; s += base_param_.sample_ds) {
    out_arclength.push_back(s);
  }
  const auto output_traj =
    motion_utils::resampleTrajectory(motion_utils::convertToTrajectory(input), out_arclength);
  auto output = motion_utils::convertToTrajectoryPointArray(output_traj);
  output.back() = input.back();  // keep the final speed.

  const size_t idx_dist = static_cast<size_t>(std::max(
    static_cast<int>((base_param_.curvature_calculation_distance) / base_param_.sample_ds), 1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(output, idx_dist);

  for (size_t i = 0; i + 1 < output.size(); i++) {
    if (fabs(curvature_v.at(i)) > base_param_.curvature_threshold) {
      // calculate the just 2 steering angle
      output.at(i).front_wheel_angle_rad = std::atan(base_param_.wheel_base * curvature_v.at(i));
      output.at(i + 1).front_wheel_angle_rad =
        std::atan(base_param_.wheel_base * curvature_v.at(i + 1));

      const double mean_vel =
        (output.at(i).longitudinal_velocity_mps + output.at(i + 1).longitudinal_velocity_mps) / 2.0;
      const double dt =
        std::max(base_param_.sample_ds / mean_vel, std::numeric_limits<double>::epsilon());
      const double steering_diff =
        fabs(output.at(i).front_wheel_angle_rad - output.at(i + 1).front_wheel_angle_rad);
      const double dt_steering =
        steering_diff / tier4_autoware_utils::deg2rad(base_param_.max_steering_angle_rate);

      if (dt_steering > dt) {
        const double target_mean_vel = (base_param_.sample_ds / dt_steering);
        for (size_t k = 0; k < 2; k++) {
          const double temp_vel =
            output.at(i + k).longitudinal_velocity_mps * (target_mean_vel / mean_vel);
          if (temp_vel < output.at(i + k).longitudinal_velocity_mps) {
            output.at(i + k).longitudinal_velocity_mps = temp_vel;
          } else {
            if (target_mean_vel < output.at(i + k).longitudinal_velocity_mps) {
              output.at(i + k).longitudinal_velocity_mps = target_mean_vel;
            }
          }
          if (output.at(i + k).longitudinal_velocity_mps < base_param_.min_curve_velocity) {
            output.at(i + k).longitudinal_velocity_mps = base_param_.min_curve_velocity;
          }
        }
      }
    }
  }

  return output;
}

}  // namespace motion_velocity_smoother
