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
#ifndef BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DEBUG_STRUCTS_HPP_
#define BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DEBUG_STRUCTS_HPP_

#include "behavior_path_lane_change_module/utils/data_structs.hpp"
#include "behavior_path_lane_change_module/utils/path.hpp"

#include <behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp>

#include <string>

namespace behavior_path_planner::data::lane_change
{
using utils::path_safety_checker::CollisionCheckDebugMap;
struct Debug
{
  std::string module_type;
  LaneChangePaths valid_paths;
  CollisionCheckDebugMap collision_check_objects;
  CollisionCheckDebugMap collision_check_objects_after_approval;
  LaneChangeTargetObjects filtered_objects;
  double collision_check_object_debug_lifetime{0.0};

  void reset()
  {
    valid_paths.clear();
    collision_check_objects.clear();
    collision_check_objects_after_approval.clear();
    filtered_objects.current_lane.clear();
    filtered_objects.target_lane.clear();
    filtered_objects.other_lane.clear();
    collision_check_object_debug_lifetime = 0.0;
  }
};
}  // namespace behavior_path_planner::data::lane_change

#endif  // BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DEBUG_STRUCTS_HPP_
