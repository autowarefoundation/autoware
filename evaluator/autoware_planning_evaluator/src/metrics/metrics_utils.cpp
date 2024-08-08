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

#include "autoware/planning_evaluator/metrics/trajectory_metrics.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
namespace planning_diagnostics
{
namespace metrics
{
namespace utils
{
using autoware::universe_utils::calcDistance2d;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

size_t getIndexAfterDistance(const Trajectory & traj, const size_t curr_id, const double distance)
{
  // Get Current Trajectory Point
  const TrajectoryPoint & curr_p = traj.points.at(curr_id);

  size_t target_id = curr_id;
  for (size_t traj_id = curr_id + 1; traj_id < traj.points.size(); ++traj_id) {
    double current_distance = calcDistance2d(traj.points.at(traj_id), curr_p);
    if (current_distance >= distance) {
      target_id = traj_id;
      break;
    }
  }

  return target_id;
}

}  // namespace utils
}  // namespace metrics
}  // namespace planning_diagnostics
