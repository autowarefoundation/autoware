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

#include "planning_evaluator/metrics/deviation_metrics.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

namespace planning_diagnostics
{
namespace metrics
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

Stat<double> calcLateralDeviation(const Trajectory & ref, const Trajectory & traj)
{
  Stat<double> stat;

  if (ref.points.empty() || traj.points.empty()) {
    return stat;
  }

  /** TODO(Maxime CLEMENT):
   * need more precise calculation, e.g., lateral distance from spline of the reference traj
   */
  for (TrajectoryPoint p : traj.points) {
    const size_t nearest_index = motion_utils::findNearestIndex(ref.points, p.pose.position);
    stat.add(
      tier4_autoware_utils::calcLateralDeviation(ref.points[nearest_index].pose, p.pose.position));
  }
  return stat;
}

Stat<double> calcYawDeviation(const Trajectory & ref, const Trajectory & traj)
{
  Stat<double> stat;

  if (ref.points.empty() || traj.points.empty()) {
    return stat;
  }

  /** TODO(Maxime CLEMENT):
   * need more precise calculation, e.g., yaw distance from spline of the reference traj
   */
  for (TrajectoryPoint p : traj.points) {
    const size_t nearest_index = motion_utils::findNearestIndex(ref.points, p.pose.position);
    stat.add(tier4_autoware_utils::calcYawDeviation(ref.points[nearest_index].pose, p.pose));
  }
  return stat;
}

Stat<double> calcVelocityDeviation(const Trajectory & ref, const Trajectory & traj)
{
  Stat<double> stat;

  if (ref.points.empty() || traj.points.empty()) {
    return stat;
  }

  // TODO(Maxime CLEMENT) need more precise calculation
  for (TrajectoryPoint p : traj.points) {
    const size_t nearest_index = motion_utils::findNearestIndex(ref.points, p.pose.position);
    stat.add(p.longitudinal_velocity_mps - ref.points[nearest_index].longitudinal_velocity_mps);
  }
  return stat;
}

}  // namespace metrics
}  // namespace planning_diagnostics
