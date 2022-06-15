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

#ifndef OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__RESAMPLE_HPP_
#define OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__RESAMPLE_HPP_

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include "boost/optional.hpp"

#include <vector>

namespace resampling
{
std::vector<rclcpp::Duration> resampledValidRelativeTimeVector(
  const rclcpp::Time & start_time, const rclcpp::Time & obj_base_time,
  const std::vector<double> & rel_time_vec, const double duration);

autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & input_path,
  const std::vector<rclcpp::Duration> & rel_time_vec);

autoware_auto_planning_msgs::msg::Trajectory applyLinearInterpolation(
  const std::vector<double> & base_index,
  const autoware_auto_planning_msgs::msg::Trajectory & base_trajectory,
  const std::vector<double> & out_index, const bool use_spline_for_pose = false);
}  // namespace resampling

#endif  // OBSTACLE_CRUISE_PLANNER__OPTIMIZATION_BASED_PLANNER__RESAMPLE_HPP_
