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
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CALCULATION_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CALCULATION_HPP_

#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"

namespace autoware::behavior_path_planner::utils::lane_change::calculation
{
using behavior_path_planner::lane_change::CommonDataPtr;

/**
 * @brief Calculates the distance from the ego vehicle to the terminal point.
 *
 * This function computes the distance from the current position of the ego vehicle
 * to either the goal pose or the end of the current lane, depending on whether
 * the vehicle's current lane is within the goal section.
 *
 * @param common_data_ptr Shared pointer to a CommonData structure, which should include:
 *  - Non null `lanes_ptr` that points to the current lanes data.
 *  - Non null `self_odometry_ptr` that contains the current pose of the ego vehicle.
 *  - Non null `route_handler_ptr` that contains the goal pose of the route.
 *
 * @return The distance to the terminal point (either the goal pose or the end of the current lane)
 * in meters.
 */
double calc_ego_dist_to_terminal_end(const CommonDataPtr & common_data_ptr);

double calc_dist_from_pose_to_terminal_end(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes,
  const Pose & src_pose);
}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CALCULATION_HPP_
