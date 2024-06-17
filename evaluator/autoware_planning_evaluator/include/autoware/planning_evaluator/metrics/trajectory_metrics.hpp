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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRICS__TRAJECTORY_METRICS_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRICS__TRAJECTORY_METRICS_HPP_

#include "autoware/planning_evaluator/stat.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

namespace planning_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

/**
 * @brief calculate relative angle metric (angle between successive points)
 * @param [in] traj input trajectory
 * @param [in] min_dist_threshold minimum distance between successive points
 * @return calculated statistics
 */
Stat<double> calcTrajectoryRelativeAngle(const Trajectory & traj, const double min_dist_threshold);

/**
 * @brief calculate metric for the distance between trajectory points
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcTrajectoryInterval(const Trajectory & traj);

/**
 * @brief calculate curvature metric
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcTrajectoryCurvature(const Trajectory & traj);

/**
 * @brief calculate length of the trajectory [m]
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcTrajectoryLength(const Trajectory & traj);

/**
 * @brief calculate duration of the trajectory [s]
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcTrajectoryDuration(const Trajectory & traj);

/**
 * @brief calculate velocity metrics for the trajectory
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcTrajectoryVelocity(const Trajectory & traj);

/**
 * @brief calculate acceleration metrics for the trajectory
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcTrajectoryAcceleration(const Trajectory & traj);

/**
 * @brief calculate jerk metrics for the trajectory
 * @param [in] traj input trajectory
 * @return calculated statistics
 */
Stat<double> calcTrajectoryJerk(const Trajectory & traj);

}  // namespace metrics
}  // namespace planning_diagnostics

#endif  // AUTOWARE__PLANNING_EVALUATOR__METRICS__TRAJECTORY_METRICS_HPP_
