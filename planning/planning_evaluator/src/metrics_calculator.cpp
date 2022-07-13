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

#include "planning_evaluator/metrics_calculator.hpp"

#include "motion_utils/motion_utils.hpp"
#include "planning_evaluator/metrics/deviation_metrics.hpp"
#include "planning_evaluator/metrics/obstacle_metrics.hpp"
#include "planning_evaluator/metrics/stability_metrics.hpp"
#include "planning_evaluator/metrics/trajectory_metrics.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

namespace planning_diagnostics
{
Stat<double> MetricsCalculator::calculate(const Metric metric, const Trajectory & traj) const
{
  // Functions to calculate metrics
  switch (metric) {
    case Metric::curvature:
      return metrics::calcTrajectoryCurvature(traj);
    case Metric::point_interval:
      return metrics::calcTrajectoryInterval(traj);
    case Metric::relative_angle:
      return metrics::calcTrajectoryRelativeAngle(traj, parameters.trajectory.min_point_dist_m);
    case Metric::length:
      return metrics::calcTrajectoryLength(traj);
    case Metric::duration:
      return metrics::calcTrajectoryDuration(traj);
    case Metric::velocity:
      return metrics::calcTrajectoryVelocity(traj);
    case Metric::acceleration:
      return metrics::calcTrajectoryAcceleration(traj);
    case Metric::jerk:
      return metrics::calcTrajectoryJerk(traj);
    case Metric::lateral_deviation:
      return metrics::calcLateralDeviation(reference_trajectory_, traj);
    case Metric::yaw_deviation:
      return metrics::calcYawDeviation(reference_trajectory_, traj);
    case Metric::velocity_deviation:
      return metrics::calcVelocityDeviation(reference_trajectory_, traj);
    case Metric::stability_frechet:
      return metrics::calcFrechetDistance(
        getLookaheadTrajectory(
          previous_trajectory_, parameters.trajectory.lookahead.max_dist_m,
          parameters.trajectory.lookahead.max_time_s),
        getLookaheadTrajectory(
          traj, parameters.trajectory.lookahead.max_dist_m,
          parameters.trajectory.lookahead.max_time_s));
    case Metric::stability:
      return metrics::calcLateralDistance(
        getLookaheadTrajectory(
          previous_trajectory_, parameters.trajectory.lookahead.max_dist_m,
          parameters.trajectory.lookahead.max_time_s),
        getLookaheadTrajectory(
          traj, parameters.trajectory.lookahead.max_dist_m,
          parameters.trajectory.lookahead.max_time_s));
    case Metric::obstacle_distance:
      return metrics::calcDistanceToObstacle(dynamic_objects_, traj);
    case Metric::obstacle_ttc:
      return metrics::calcTimeToCollision(dynamic_objects_, traj, parameters.obstacle.dist_thr_m);
    default:
      throw std::runtime_error(
        "[MetricsCalculator][calculate()] unknown Metric " +
        std::to_string(static_cast<int>(metric)));
  }
}

void MetricsCalculator::setReferenceTrajectory(const Trajectory & traj)
{
  reference_trajectory_ = traj;
}

void MetricsCalculator::setPreviousTrajectory(const Trajectory & traj)
{
  previous_trajectory_ = traj;
}

void MetricsCalculator::setPredictedObjects(const PredictedObjects & objects)
{
  dynamic_objects_ = objects;
}

void MetricsCalculator::setEgoPose(const geometry_msgs::msg::Pose & pose) { ego_pose_ = pose; }

Trajectory MetricsCalculator::getLookaheadTrajectory(
  const Trajectory & traj, const double max_dist_m, const double max_time_s) const
{
  if (traj.points.empty()) {
    return traj;
  }

  const auto ego_index = motion_utils::findNearestSegmentIndex(traj.points, ego_pose_.position);
  Trajectory lookahead_traj;
  lookahead_traj.header = traj.header;
  double dist = 0.0;
  double time = 0.0;
  auto curr_point_it = std::next(traj.points.begin(), ego_index);
  auto prev_point_it = curr_point_it;
  while (curr_point_it != traj.points.end() && dist <= max_dist_m && time <= max_time_s) {
    lookahead_traj.points.push_back(*curr_point_it);
    const auto d = tier4_autoware_utils::calcDistance2d(
      prev_point_it->pose.position, curr_point_it->pose.position);
    dist += d;
    if (prev_point_it->longitudinal_velocity_mps != 0.0) {
      time += d / std::abs(prev_point_it->longitudinal_velocity_mps);
    }
    prev_point_it = curr_point_it;
    ++curr_point_it;
  }
  return lookahead_traj;
}

}  // namespace planning_diagnostics
