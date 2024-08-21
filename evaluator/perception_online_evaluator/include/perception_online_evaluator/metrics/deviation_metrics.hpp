// Copyright 2024 TIER IV, Inc.
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

#ifndef PERCEPTION_ONLINE_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_
#define PERCEPTION_ONLINE_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_

#include "perception_online_evaluator/stat.hpp"

#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace perception_diagnostics
{
namespace metrics
{
using autoware_perception_msgs::msg::PredictedPath;
using geometry_msgs::msg::Pose;

/**
 * @brief calculate lateral deviation of the given path from the reference path
 * @param [in] ref_path reference path
 * @param [in] pred_path predicted path
 * @return calculated statistics
 */
double calcLateralDeviation(const std::vector<Pose> & ref_path, const Pose & target_pose);

/**
 * @brief calculate yaw deviation of the given path from the reference path
 * @param [in] ref_path reference path
 * @param [in] pred_path predicted path
 * @return calculated statistics
 */
double calcYawDeviation(const std::vector<Pose> & ref_path, const Pose & target_pose);

}  // namespace metrics
}  // namespace perception_diagnostics

#endif  // PERCEPTION_ONLINE_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_
