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
using behavior_path_planner::lane_change::LCParamPtr;

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

/**
 * @brief Calculates the minimum stopping distance to terminal start.
 *
 * This function computes the minimum stopping distance to terminal start based on the
 * minimum lane changing velocity and the minimum longitudinal acceleration. It then
 * compares this calculated distance with a pre-defined backward length buffer parameter
 * and returns the larger of the two values to ensure safe lane changing.
 *
 * @param lc_param_ptr Shared pointer to an LCParam structure, which should include:
 *  - `minimum_lane_changing_velocity`: The minimum velocity required for lane changing.
 *  - `min_longitudinal_acc`: The minimum longitudinal acceleration used for deceleration.
 *  - `backward_length_buffer_for_end_of_lane`: A predefined backward buffer length parameter.
 *
 * @return The required backward buffer distance in meters.
 */
double calc_stopping_distance(const LCParamPtr & lc_param_ptr);
}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__CALCULATION_HPP_
