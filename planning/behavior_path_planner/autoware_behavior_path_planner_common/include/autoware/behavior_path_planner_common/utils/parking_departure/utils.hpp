// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__UTILS_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/parking_departure/common_module_data.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::utils::parking_departure
{

using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using autoware::behavior_path_planner::utils::path_safety_checker::EgoPredictedPathParams;
using autoware::behavior_path_planner::utils::path_safety_checker::ObjectsFilteringParams;
using autoware::behavior_path_planner::utils::path_safety_checker::SafetyCheckParams;

std::optional<double> calcFeasibleDecelDistance(
  std::shared_ptr<const PlannerData> planner_data, const double acc_lim, const double jerk_lim,
  const double target_velocity);

/**
 * @brief Update path velocities based on driving direction.
 *
 * This function updates the longitudinal velocity of each point in the provided paths,
 * based on whether the vehicle is driving forward or backward. It also sets the terminal
 * velocity and acceleration for each path.
 *
 * @param paths A vector of paths with lane IDs to be updated.
 * @param terminal_vel_acc_pairs A vector of pairs, where each pair contains the terminal
 *                               velocity and acceleration for a corresponding path.
 * @param target_velocity The target velocity for ego vehicle predicted path.
 * @param acceleration The acceleration for ego vehicle predicted path.
 */
void modifyVelocityByDirection(
  std::vector<PathWithLaneId> & paths,
  std::vector<std::pair<double, double>> & terminal_vel_acc_pairs, const double target_velocity,
  const double acceleration);

void updatePathProperty(
  const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::pair<double, double> & pairs_terminal_velocity_and_accel);

void initializeCollisionCheckDebugMap(CollisionCheckDebugMap & collision_check_debug_map);

void updateSafetyCheckTargetObjectsData(
  StartGoalPlannerData & data, const PredictedObjects & filtered_objects,
  const TargetObjectsOnLane & target_objects_on_lane,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path);

std::pair<double, double> getPairsTerminalVelocityAndAccel(
  const std::vector<std::pair<double, double>> & pairs_terminal_velocity_and_accel,
  const size_t current_path_idx);

std::optional<PathWithLaneId> generateFeasibleStopPath(
  PathWithLaneId & current_path, std::shared_ptr<const PlannerData> planner_data,
  std::optional<geometry_msgs::msg::Pose> & stop_pose, const double maximum_deceleration,
  const double maximum_jerk);

/**
 * @brief calculate end arc length to generate reference path considering the goal position
 * @return a pair of s_end and terminal_is_goal
 */
std::pair<double, bool> calcEndArcLength(
  const double s_start, const double forward_path_length, const lanelet::ConstLanelets & road_lanes,
  const Pose & goal_pose);

}  // namespace autoware::behavior_path_planner::utils::parking_departure

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__UTILS_HPP_
