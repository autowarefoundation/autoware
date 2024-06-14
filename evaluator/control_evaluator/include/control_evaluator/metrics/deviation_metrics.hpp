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

#ifndef CONTROL_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_
#define CONTROL_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

namespace control_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

/**
 * @brief calculate lateral deviation of the given trajectory from the reference trajectory
 * @param [in] ref reference trajectory
 * @param [in] point input point
 * @return lateral deviation
 */
double calcLateralDeviation(const Trajectory & traj, const Point & point);

/**
 * @brief calculate yaw deviation of the given trajectory from the reference trajectory
 * @param [in] traj input trajectory
 * @param [in] pose input pose
 * @return yaw deviation
 */
double calcYawDeviation(const Trajectory & traj, const Pose & pose);

}  // namespace metrics
}  // namespace control_diagnostics

#endif  // CONTROL_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_
