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

#ifndef OBJECT_RECOGNITION_UTILS__PREDICTED_PATH_UTILS_HPP_
#define OBJECT_RECOGNITION_UTILS__PREDICTED_PATH_UTILS_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/optional.hpp>

#include <vector>

namespace object_recognition_utils
{
/**
 * @brief Calculate Interpolated Pose from predicted path by time
 * @param path Input predicted path
 * @param relative_time time at interpolated point. This should be within [0.0,
 * time_step*(num_of_path_points)]
 * @return interpolated pose
 */
boost::optional<geometry_msgs::msg::Pose> calcInterpolatedPose(
  const autoware_auto_perception_msgs::msg::PredictedPath & path, const double relative_time);

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
  const bool use_spline_for_z = false);

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
  const bool use_spline_for_xy = true, const bool use_spline_for_z = false);
}  // namespace object_recognition_utils

#endif  // OBJECT_RECOGNITION_UTILS__PREDICTED_PATH_UTILS_HPP_
