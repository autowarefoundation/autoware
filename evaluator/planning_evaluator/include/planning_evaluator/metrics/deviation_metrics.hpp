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

#ifndef PLANNING_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_
#define PLANNING_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_

#include "planning_evaluator/stat.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

namespace planning_diagnostics
{
namespace metrics
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

/**
 * @brief calculate lateral deviation of the given trajectory from the reference trajectory
 * @param [in] ref reference trajectory
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcLateralDeviation(const Trajectory & ref, const Trajectory & traj);

/**
 * @brief calculate yaw deviation of the given trajectory from the reference trajectory
 * @param [in] ref reference trajectory
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcYawDeviation(const Trajectory & ref, const Trajectory & traj);

/**
 * @brief calculate velocity deviation of the given trajectory from the reference trajectory
 * @param [in] ref reference trajectory
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcVelocityDeviation(const Trajectory & ref, const Trajectory & traj);

/**
 * @brief calculate longitudinal deviation of the given ego pose from the modified goal pose
 * @param [in] base_pose base pose
 * @param [in] target_point target point
 * @return calculated statistics
 */
Stat<double> calcLongitudinalDeviation(const Pose & base_pose, const Point & target_point);

/**
 * @brief calculate lateral deviation of the given ego pose from the modified goal pose
 * @param [in] base_pose base pose
 * @param [in] target_point target point
 * @return calculated statistics
 */
Stat<double> calcLateralDeviation(const Pose & base_pose, const Point & target_point);

/**
 * @brief calculate yaw deviation of the given ego pose from the modified goal pose
 * @param [in] base_pose base pose
 * @param [in] target_pose target pose
 * @return calculated statistics
 */
Stat<double> calcYawDeviation(const Pose & base_pose, const Pose & target_pose);

}  // namespace metrics
}  // namespace planning_diagnostics

#endif  // PLANNING_EVALUATOR__METRICS__DEVIATION_METRICS_HPP_
