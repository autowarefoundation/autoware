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

#include "planning_evaluator/metrics/stability_metrics.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <Eigen/Core>

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include <algorithm>

namespace planning_diagnostics
{
namespace metrics
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

Stat<double> calcFrechetDistance(const Trajectory & traj1, const Trajectory & traj2)
{
  Stat<double> stat;

  if (traj1.points.empty() || traj2.points.empty()) {
    return stat;
  }

  Eigen::MatrixXd ca = Eigen::MatrixXd::Zero(traj1.points.size(), traj2.points.size());

  for (size_t i = 0; i < traj1.points.size(); ++i) {
    for (size_t j = 0; j < traj2.points.size(); ++j) {
      const double dist = tier4_autoware_utils::calcDistance2d(traj1.points[i], traj2.points[j]);
      if (i > 0 && j > 0) {
        ca(i, j) = std::max(std::min(ca(i - 1, j), std::min(ca(i - 1, j - 1), ca(i, j - 1))), dist);
      } else if (i > 0 /*&& j == 0*/) {
        ca(i, j) = std::max(ca(i - 1, 0), dist);
      } else if (j > 0 /*&& i == 0*/) {
        ca(i, j) = std::max(ca(0, j - 1), dist);
      } else { /* i == j == 0 */
        ca(i, j) = dist;
      }
    }
  }
  stat.add(ca(traj1.points.size() - 1, traj2.points.size() - 1));
  return stat;
}

Stat<double> calcLateralDistance(const Trajectory & traj1, const Trajectory & traj2)
{
  Stat<double> stat;
  if (traj1.points.empty()) {
    return stat;
  }
  for (const auto & point : traj2.points) {
    const auto p0 = tier4_autoware_utils::getPoint(point);
    // find nearest segment
    const size_t nearest_segment_idx = motion_utils::findNearestSegmentIndex(traj1.points, p0);
    double dist;
    // distance to segment
    if (
      nearest_segment_idx == traj1.points.size() - 2 &&
      motion_utils::calcLongitudinalOffsetToSegment(traj1.points, nearest_segment_idx, p0) >
        tier4_autoware_utils::calcDistance2d(
          traj1.points[nearest_segment_idx], traj1.points[nearest_segment_idx + 1])) {
      // distance to last point
      dist = tier4_autoware_utils::calcDistance2d(traj1.points.back(), p0);
    } else if (  // NOLINT
      nearest_segment_idx == 0 &&
      motion_utils::calcLongitudinalOffsetToSegment(traj1.points, nearest_segment_idx, p0) <= 0) {
      // distance to first point
      dist = tier4_autoware_utils::calcDistance2d(traj1.points.front(), p0);
    } else {
      // orthogonal distance
      const auto p1 = tier4_autoware_utils::getPoint(traj1.points[nearest_segment_idx]);
      const auto p2 = tier4_autoware_utils::getPoint(traj1.points[nearest_segment_idx + 1]);
      dist = std::abs((p2.x - p1.x) * (p1.y - p0.y) - (p1.x - p0.x) * (p2.y - p1.y)) /
             std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
    }
    stat.add(dist);
  }
  return stat;
}

}  // namespace metrics
}  // namespace planning_diagnostics
