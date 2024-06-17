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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__COMMON_MODULE_DATA_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__COMMON_MODULE_DATA_HPP_

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
using autoware_perception_msgs::msg::PredictedObjects;
/*
 * Common data for start/goal_planner module
 */
struct StartGoalPlannerData
{
  // filtered objects
  PredictedObjects filtered_objects;
  TargetObjectsOnLane target_objects_on_lane;
  std::vector<PoseWithVelocityStamped> ego_predicted_path;
  // collision check debug map
  CollisionCheckDebugMap collision_check;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__COMMON_MODULE_DATA_HPP_
