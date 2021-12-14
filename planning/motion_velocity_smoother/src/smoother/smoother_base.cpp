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

#include "motion_velocity_smoother/resample.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace motion_velocity_smoother
{
void SmootherBase::setParam(const BaseParam & param) { base_param_ = param; }

double SmootherBase::getMaxAccel() const { return base_param_.max_accel; }

double SmootherBase::getMinDecel() const { return base_param_.min_decel; }

double SmootherBase::getMaxJerk() const { return base_param_.max_jerk; }

double SmootherBase::getMinJerk() const { return base_param_.min_jerk; }

boost::optional<TrajectoryPoints> SmootherBase::applyLateralAccelerationFilter(
  const TrajectoryPoints & input) const
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
  const std::vector<double> in_arclength = trajectory_utils::calcArclengthArray(input);
  for (double s = 0; s < in_arclength.back(); s += points_interval) {
    out_arclength.push_back(s);
  }
  auto output = trajectory_utils::applyLinearInterpolation(in_arclength, input, out_arclength);
  if (!output) {
    RCLCPP_WARN(
      rclcpp::get_logger("smoother").get_child("smoother_base"),
      "interpolation failed at lateral acceleration filter.");
    return boost::none;
  }
  output->back() = input.back();  // keep the final speed.

  constexpr double curvature_calc_dist = 5.0;  // [m] calc curvature with 5m away points
  const size_t idx_dist =
    static_cast<size_t>(std::max(static_cast<int>((curvature_calc_dist) / points_interval), 1));

  // Calculate curvature assuming the trajectory points interval is constant
  const auto curvature_v = trajectory_utils::calcTrajectoryCurvatureFrom3Points(*output, idx_dist);
  if (!curvature_v) {
    return boost::optional<TrajectoryPoints>(input);
  }

  //  Decrease speed according to lateral G
  const size_t before_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_before_curve / points_interval));
  const size_t after_decel_index =
    static_cast<size_t>(std::round(base_param_.decel_distance_after_curve / points_interval));
  const double max_lateral_accel_abs = std::fabs(base_param_.max_lateral_accel);

  for (size_t i = 0; i < output->size(); ++i) {
    double curvature = 0.0;
    const size_t start = i > after_decel_index ? i - after_decel_index : 0;
    const size_t end = std::min(output->size(), i + before_decel_index);
    for (size_t j = start; j < end; ++j) {
      curvature = std::max(curvature, std::fabs(curvature_v->at(j)));
    }
    double v_curvature_max = std::sqrt(max_lateral_accel_abs / std::max(curvature, 1.0E-5));
    v_curvature_max = std::max(v_curvature_max, base_param_.min_curve_velocity);
    if (output->at(i).longitudinal_velocity_mps > v_curvature_max) {
      output->at(i).longitudinal_velocity_mps = v_curvature_max;
    }
  }
  return output;
}

}  // namespace motion_velocity_smoother
