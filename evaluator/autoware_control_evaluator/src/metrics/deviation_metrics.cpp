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

#include "autoware/control_evaluator/metrics/deviation_metrics.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/geometry/pose_deviation.hpp"

namespace control_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::Trajectory;

double calcLateralDeviation(const Trajectory & traj, const Point & point)
{
  const size_t nearest_index = autoware::motion_utils::findNearestIndex(traj.points, point);
  return std::abs(
    autoware::universe_utils::calcLateralDeviation(traj.points[nearest_index].pose, point));
}

double calcYawDeviation(const Trajectory & traj, const Pose & pose)
{
  const size_t nearest_index = autoware::motion_utils::findNearestIndex(traj.points, pose.position);
  return std::abs(
    autoware::universe_utils::calcYawDeviation(traj.points[nearest_index].pose, pose));
}

}  // namespace metrics
}  // namespace control_diagnostics
