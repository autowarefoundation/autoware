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

#include "planning_evaluator/metrics/trajectory_metrics.hpp"

#include "planning_evaluator/metrics/metrics_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
namespace planning_diagnostics
{
namespace metrics
{
using tier4_autoware_utils::calcCurvature;
using tier4_autoware_utils::calcDistance2d;

Stat<double> calcTrajectoryInterval(const Trajectory & traj)
{
  Stat<double> stat;
  for (size_t i = 1; i < traj.points.size(); ++i) {
    stat.add(calcDistance2d(traj.points.at(i), traj.points.at(i - 1)));
  }
  return stat;
}

Stat<double> calcTrajectoryRelativeAngle(const Trajectory & traj, const double min_dist_threshold)
{
  Stat<double> stat;
  // We need at least three points to compute relative angle
  const size_t relative_angle_points_num = 3;
  if (traj.points.size() < relative_angle_points_num) {
    return stat;
  }

  for (size_t p1_id = 0; p1_id <= traj.points.size() - relative_angle_points_num; ++p1_id) {
    // Get Point1
    const auto & p1 = traj.points.at(p1_id).pose.position;

    // Get Point2
    const auto & p2 = traj.points.at(p1_id + 1).pose.position;

    // Get Point3
    const auto & p3 = traj.points.at(p1_id + 2).pose.position;

    // ignore invert driving direction
    if (
      traj.points.at(p1_id).longitudinal_velocity_mps < 0 ||
      traj.points.at(p1_id + 1).longitudinal_velocity_mps < 0 ||
      traj.points.at(p1_id + 2).longitudinal_velocity_mps < 0) {
      continue;
    }

    // convert to p1 coordinate
    const double x3 = p3.x - p1.x;
    const double x2 = p2.x - p1.x;
    const double y3 = p3.y - p1.y;
    const double y2 = p2.y - p1.y;

    // skip too close points case
    if (std::hypot(x3, y3) < min_dist_threshold || std::hypot(x2, y2) < min_dist_threshold) {
      continue;
    }

    // calculate relative angle of vector p3 based on p1p2 vector
    const double th = std::atan2(y2, x2);
    const double th2 =
      std::atan2(-x3 * std::sin(th) + y3 * std::cos(th), x3 * std::cos(th) + y3 * std::sin(th));
    stat.add(th2);
  }
  return stat;
}

Stat<double> calcTrajectoryCurvature(const Trajectory & traj)
{
  Stat<double> stat;
  // We need at least three points to compute curvature
  if (traj.points.size() < 3) {
    return stat;
  }

  constexpr double points_distance = 1.0;

  for (size_t p1_id = 0; p1_id < traj.points.size() - 2; ++p1_id) {
    // Get Point1
    const auto p1 = traj.points.at(p1_id).pose.position;

    // Get Point2
    const auto p2_id = utils::getIndexAfterDistance(traj, p1_id, points_distance);
    const auto p2 = traj.points.at(p2_id).pose.position;

    // Get Point3
    const auto p3_id = utils::getIndexAfterDistance(traj, p2_id, points_distance);
    const auto p3 = traj.points.at(p3_id).pose.position;

    // no need to check for pi, since there is no point with "points_distance" from p1.
    if (p1_id == p2_id || p1_id == p3_id || p2_id == p3_id) {
      break;
    }

    stat.add(calcCurvature(p1, p2, p3));
  }
  return stat;
}

Stat<double> calcTrajectoryLength(const Trajectory & traj)
{
  double length = 0.0;
  for (size_t i = 1; i < traj.points.size(); ++i) {
    length += calcDistance2d(traj.points.at(i), traj.points.at(i - 1));
  }
  Stat<double> stat;
  stat.add(length);
  return stat;
}

Stat<double> calcTrajectoryDuration(const Trajectory & traj)
{
  double duration = 0.0;
  for (size_t i = 0; i < traj.points.size() - 1; ++i) {
    const double length = calcDistance2d(traj.points.at(i), traj.points.at(i + 1));
    const double velocity = traj.points.at(i).longitudinal_velocity_mps;
    if (velocity != 0) {
      duration += length / std::abs(velocity);
    }
  }
  Stat<double> stat;
  stat.add(duration);
  return stat;
}

Stat<double> calcTrajectoryVelocity(const Trajectory & traj)
{
  Stat<double> stat;
  for (TrajectoryPoint p : traj.points) {
    stat.add(p.longitudinal_velocity_mps);
  }
  return stat;
}

Stat<double> calcTrajectoryAcceleration(const Trajectory & traj)
{
  Stat<double> stat;
  for (TrajectoryPoint p : traj.points) {
    stat.add(p.acceleration_mps2);
  }
  return stat;
}

Stat<double> calcTrajectoryJerk(const Trajectory & traj)
{
  Stat<double> stat;
  for (size_t i = 0; i < traj.points.size() - 1; ++i) {
    const double vel = traj.points.at(i).longitudinal_velocity_mps;
    if (vel != 0) {
      const double duration =
        calcDistance2d(traj.points.at(i), traj.points.at(i + 1)) / std::abs(vel);
      if (duration != 0) {
        const double start_accel = traj.points.at(i).acceleration_mps2;
        const double end_accel = traj.points.at(i + 1).acceleration_mps2;
        stat.add(end_accel - start_accel / duration);
      }
    }
  }
  return stat;
}
}  // namespace metrics
}  // namespace planning_diagnostics
