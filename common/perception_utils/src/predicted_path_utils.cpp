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

#include "perception_utils/predicted_path_utils.hpp"

#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spherical_linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"

#include <algorithm>

namespace perception_utils
{
boost::optional<geometry_msgs::msg::Pose> calcInterpolatedPose(
  const autoware_auto_perception_msgs::msg::PredictedPath & path, const double relative_time)
{
  // Check if relative time is in the valid range
  if (path.path.empty() || relative_time < 0.0) {
    return boost::none;
  }

  constexpr double epsilon = 1e-6;
  const double & time_step = rclcpp::Duration(path.time_step).seconds();
  for (size_t path_idx = 1; path_idx < path.path.size(); ++path_idx) {
    const auto & pt = path.path.at(path_idx);
    const auto & prev_pt = path.path.at(path_idx - 1);
    if (relative_time - epsilon < time_step * path_idx) {
      const double offset = relative_time - time_step * (path_idx - 1);
      const double ratio = std::clamp(offset / time_step, 0.0, 1.0);
      return tier4_autoware_utils::calcInterpolatedPose(prev_pt, pt, ratio, false);
    }
  }

  return boost::none;
}

autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & path,
  const std::vector<double> & resampled_time, const bool use_spline_for_xy,
  const bool use_spline_for_z)
{
  if (path.path.empty() || resampled_time.empty()) {
    throw std::invalid_argument("input path or resampled_time is empty");
  }

  const double & time_step = rclcpp::Duration(path.time_step).seconds();
  std::vector<double> input_time(path.path.size());
  std::vector<double> x(path.path.size());
  std::vector<double> y(path.path.size());
  std::vector<double> z(path.path.size());
  std::vector<geometry_msgs::msg::Quaternion> quat(path.path.size());
  for (size_t i = 0; i < path.path.size(); ++i) {
    input_time.at(i) = time_step * i;
    x.at(i) = path.path.at(i).position.x;
    y.at(i) = path.path.at(i).position.y;
    z.at(i) = path.path.at(i).position.z;
    quat.at(i) = path.path.at(i).orientation;
  }

  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_time, input, resampled_time);
  };
  const auto spline = [&](const auto & input) {
    return interpolation::spline(input_time, input, resampled_time);
  };
  const auto slerp = [&](const auto & input) {
    return interpolation::slerp(input_time, input, resampled_time);
  };

  const auto interpolated_x = use_spline_for_xy ? spline(x) : lerp(x);
  const auto interpolated_y = use_spline_for_xy ? spline(y) : lerp(y);
  const auto interpolated_z = use_spline_for_z ? spline(z) : lerp(z);
  const auto interpolated_quat = slerp(quat);

  autoware_auto_perception_msgs::msg::PredictedPath resampled_path;
  const auto resampled_size = std::min(resampled_path.path.max_size(), resampled_time.size());
  resampled_path.confidence = path.confidence;
  resampled_path.path.resize(resampled_size);

  // Set Position
  for (size_t i = 0; i < resampled_size; ++i) {
    const auto p = tier4_autoware_utils::createPoint(
      interpolated_x.at(i), interpolated_y.at(i), interpolated_z.at(i));
    resampled_path.path.at(i).position = p;
    resampled_path.path.at(i).orientation = interpolated_quat.at(i);
  }

  return resampled_path;
}

autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & path,
  const double sampling_time_interval, const double sampling_horizon, const bool use_spline_for_xy,
  const bool use_spline_for_z)
{
  if (path.path.empty()) {
    throw std::invalid_argument("Predicted Path is empty");
  }

  if (sampling_time_interval <= 0.0 || sampling_horizon <= 0.0) {
    throw std::invalid_argument("sampling time interval or sampling time horizon is negative");
  }

  // Calculate Horizon
  const double predicted_horizon =
    rclcpp::Duration(path.time_step).seconds() * static_cast<double>(path.path.size() - 1);
  const double horizon = std::min(predicted_horizon, sampling_horizon);

  // Get sampling time vector
  constexpr double epsilon = 1e-6;
  std::vector<double> sampling_time_vector;
  for (double t = 0.0; t < horizon + epsilon; t += sampling_time_interval) {
    sampling_time_vector.push_back(t);
  }

  // Resample and substitute time interval
  auto resampled_path =
    resamplePredictedPath(path, sampling_time_vector, use_spline_for_xy, use_spline_for_z);
  resampled_path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval);
  return resampled_path;
}
}  // namespace perception_utils
