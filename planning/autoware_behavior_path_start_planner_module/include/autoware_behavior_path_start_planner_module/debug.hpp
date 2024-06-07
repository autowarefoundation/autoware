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

#ifndef AUTOWARE_BEHAVIOR_PATH_START_PLANNER_MODULE__DEBUG_HPP_
#define AUTOWARE_BEHAVIOR_PATH_START_PLANNER_MODULE__DEBUG_HPP_

#include "autoware_behavior_path_start_planner_module/data_structs.hpp"

#include <string>
#include <vector>

namespace behavior_path_planner
{
using behavior_path_planner::StartPlannerDebugData;

void updateSafetyCheckDebugData(
  StartPlannerDebugData & data, const PredictedObjects & filtered_objects,
  const TargetObjectsOnLane & target_objects_on_lane,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path)
{
  data.filtered_objects = filtered_objects;
  data.target_objects_on_lane = target_objects_on_lane;
  data.ego_predicted_path = ego_predicted_path;
}

}  // namespace behavior_path_planner

#endif  // AUTOWARE_BEHAVIOR_PATH_START_PLANNER_MODULE__DEBUG_HPP_
