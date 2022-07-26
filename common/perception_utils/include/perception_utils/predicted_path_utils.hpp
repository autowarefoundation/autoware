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

#ifndef PERCEPTION_UTILS__PREDICTED_PATH_UTILS_HPP_
#define PERCEPTION_UTILS__PREDICTED_PATH_UTILS_HPP_

#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "perception_utils/geometry.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"
#include <geometry_msgs/msg/pose.hpp>

#include <boost/optional.hpp>

#include <algorithm>
#include <vector>

namespace perception_utils
{
/**
 * @brief Calculate Interpolated Pose from predicted path by time
 * @param path Input predicted path
 * @param relative_time time at interpolated point. This should be within [0.0,
 * time_step*(num_of_path_points)]
 * @return interpolated pose
 */
inline boost::optional<geometry_msgs::msg::Pose> calcInterpolatedPose(
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
      return tier4_autoware_utils::calcInterpolatedPose(prev_pt, pt, ratio);
    }
  }

  return boost::none;
}

/**
 * @brief Resampling predicted path by time step vector. Note this function does not substitute
 * original time step
 * @param path Input predicted path
 * @param resampled_time resampled time at each resampling point. Each time should be within [0.0,
 * time_step*(num_of_path_points)]
 * @return resampled path
 */
autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & path,
  const std::vector<double> & resampled_time, const bool use_spline_for_xy = true,
  const bool use_spline_for_z = false)
{
  if (path.path.empty() || resampled_time.empty()) {
    throw std::invalid_argument("input path or resampled_time is empty");
  }

  const double & time_step = rclcpp::Duration(path.time_step).seconds();
  std::vector<double> input_time(path.path.size());
  std::vector<double> x(path.path.size());
  std::vector<double> y(path.path.size());
  std::vector<double> z(path.path.size());
  for (size_t i = 0; i < path.path.size(); ++i) {
    input_time.at(i) = time_step * i;
    x.at(i) = path.path.at(i).position.x;
    y.at(i) = path.path.at(i).position.y;
    z.at(i) = path.path.at(i).position.z;
  }

  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_time, input, resampled_time);
  };
  const auto slerp = [&](const auto & input) {
    return interpolation::slerp(input_time, input, resampled_time);
  };

  const auto interpolated_x = use_spline_for_xy ? slerp(x) : lerp(x);
  const auto interpolated_y = use_spline_for_xy ? slerp(y) : lerp(y);
  const auto interpolated_z = use_spline_for_z ? slerp(z) : lerp(z);

  autoware_auto_perception_msgs::msg::PredictedPath resampled_path;
  const auto resampled_size = std::min(resampled_path.path.max_size(), resampled_time.size());
  resampled_path.confidence = path.confidence;
  resampled_path.path.resize(resampled_size);

  // Set Position
  for (size_t i = 0; i < resampled_size; ++i) {
    const auto p = tier4_autoware_utils::createPoint(
      interpolated_x.at(i), interpolated_y.at(i), interpolated_z.at(i));
    resampled_path.path.at(i).position = p;
  }

  // Set Quaternion
  for (size_t i = 0; i < resampled_size - 1; ++i) {
    const auto & src_point = resampled_path.path.at(i).position;
    const auto & dst_point = resampled_path.path.at(i + 1).position;
    const double pitch = tier4_autoware_utils::calcElevationAngle(src_point, dst_point);
    const double yaw = tier4_autoware_utils::calcAzimuthAngle(src_point, dst_point);
    resampled_path.path.at(i).orientation =
      tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw);
  }
  resampled_path.path.back().orientation = resampled_path.path.at(resampled_size - 2).orientation;

  return resampled_path;
}

/**
 * @brief Resampling predicted path by sampling time interval. Note that this function samples
 * terminal point as well as points by sampling time interval
 * @param path Input predicted path
 * @param sampling_time_interval sampling time interval for each point
 * @param sampling_horizon sampling time horizon
 * @return resampled path
 */
autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & path,
  const double sampling_time_interval, const double sampling_horizon,
  const bool use_spline_for_xy = true, const bool use_spline_for_z = false)
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

#endif  // PERCEPTION_UTILS__PREDICTED_PATH_UTILS_HPP_
