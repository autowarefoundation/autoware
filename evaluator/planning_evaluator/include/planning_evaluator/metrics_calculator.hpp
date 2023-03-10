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

#ifndef PLANNING_EVALUATOR__METRICS_CALCULATOR_HPP_
#define PLANNING_EVALUATOR__METRICS_CALCULATOR_HPP_

#include "planning_evaluator/metrics/metric.hpp"
#include "planning_evaluator/parameters.hpp"
#include "planning_evaluator/stat.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace planning_diagnostics
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

class MetricsCalculator
{
public:
  Parameters parameters;

  MetricsCalculator() = default;

  /**
   * @brief calculate
   * @param [in] metric Metric enum value
   * @return string describing the requested metric
   */
  Stat<double> calculate(const Metric metric, const Trajectory & traj) const;

  /**
   * @brief set the reference trajectory used to calculate the deviation metrics
   * @param [in] traj input reference trajectory
   */
  void setReferenceTrajectory(const Trajectory & traj);

  /**
   * @brief set the previous trajectory used to calculate the stability metrics
   * @param [in] traj input previous trajectory
   */
  void setPreviousTrajectory(const Trajectory & traj);

  /**
   * @brief set the dynamic objects used to calculate obstacle metrics
   * @param [in] traj input previous trajectory
   */
  void setPredictedObjects(const PredictedObjects & objects);

  /**
   * @brief set the ego pose
   * @param [in] traj input previous trajectory
   */
  void setEgoPose(const geometry_msgs::msg::Pose & pose);

private:
  /**
   * @brief trim a trajectory from the current ego pose to some fixed time or distance
   * @param [in] traj input trajectory to trim
   * @param [in] max_dist_m [m] maximum distance ahead of the ego pose
   * @param [in] max_time_s [s] maximum time ahead of the ego pose
   * @return sub-trajectory starting from the ego pose and of maximum length max_dist_m, maximum
   * duration max_time_s
   */
  Trajectory getLookaheadTrajectory(
    const Trajectory & traj, const double max_dist_m, const double max_time_s) const;

  Trajectory reference_trajectory_;
  Trajectory reference_trajectory_lookahead_;
  Trajectory previous_trajectory_;
  Trajectory previous_trajectory_lookahead_;
  PredictedObjects dynamic_objects_;
  geometry_msgs::msg::Pose ego_pose_;
};  // class MetricsCalculator

}  // namespace planning_diagnostics

#endif  // PLANNING_EVALUATOR__METRICS_CALCULATOR_HPP_
