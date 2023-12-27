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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__COMMON_MODULE_DATA_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__COMMON_MODULE_DATA_HPP_

#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <vector>

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
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

  Pose refined_start_pose;
  std::vector<Pose> start_pose_candidates;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__COMMON_MODULE_DATA_HPP_
