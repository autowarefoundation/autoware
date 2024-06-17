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

#ifndef AUTOWARE__PLANNING_EVALUATOR__METRICS__METRICS_UTILS_HPP_
#define AUTOWARE__PLANNING_EVALUATOR__METRICS__METRICS_UTILS_HPP_

#include "autoware/planning_evaluator/stat.hpp"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

namespace planning_diagnostics
{
namespace metrics
{
namespace utils
{
using autoware_planning_msgs::msg::Trajectory;

/**
 * @brief find the index in the trajectory at the given distance of the given index
 * @param [in] traj input trajectory
 * @param [in] curr_id index
 * @param [in] distance distance
 * @return index of the trajectory point at distance ahead of traj[curr_id]
 */
size_t getIndexAfterDistance(const Trajectory & traj, const size_t curr_id, const double distance);

}  // namespace utils
}  // namespace metrics
}  // namespace planning_diagnostics
#endif  // AUTOWARE__PLANNING_EVALUATOR__METRICS__METRICS_UTILS_HPP_
