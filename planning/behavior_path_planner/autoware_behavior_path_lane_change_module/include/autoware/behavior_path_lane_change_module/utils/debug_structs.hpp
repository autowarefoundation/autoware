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
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DEBUG_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DEBUG_STRUCTS_HPP_

#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"

#include <autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp>

#include <geometry_msgs/msg/detail/polygon__struct.hpp>

#include <lanelet2_core/Forward.h>

#include <limits>
#include <string>

namespace autoware::behavior_path_planner::lane_change
{
using utils::path_safety_checker::CollisionCheckDebugMap;
struct Debug
{
  std::string module_type;
  LaneChangePaths valid_paths;
  CollisionCheckDebugMap collision_check_objects;
  CollisionCheckDebugMap collision_check_objects_after_approval;
  FilteredByLanesExtendedObjects filtered_objects;
  geometry_msgs::msg::Polygon execution_area;
  geometry_msgs::msg::Pose ego_pose;
  lanelet::ConstLanelets current_lanes;
  lanelet::ConstLanelets target_lanes;
  lanelet::ConstLanelets target_backward_lanes;
  double collision_check_object_debug_lifetime{0.0};
  double distance_to_end_of_current_lane{std::numeric_limits<double>::max()};
  double distance_to_lane_change_finished{std::numeric_limits<double>::max()};
  double distance_to_abort_finished{std::numeric_limits<double>::max()};
  bool is_able_to_return_to_current_lane{false};
  bool is_stuck{false};
  bool is_abort{false};

  void reset()
  {
    valid_paths.clear();
    collision_check_objects.clear();
    collision_check_objects_after_approval.clear();
    filtered_objects.current_lane.clear();
    filtered_objects.target_lane_leading.clear();
    filtered_objects.target_lane_trailing.clear();
    filtered_objects.other_lane.clear();
    execution_area.points.clear();
    current_lanes.clear();
    target_lanes.clear();
    target_backward_lanes.clear();

    collision_check_object_debug_lifetime = 0.0;
    distance_to_end_of_current_lane = std::numeric_limits<double>::max();
    distance_to_lane_change_finished = std::numeric_limits<double>::max();
    distance_to_abort_finished = std::numeric_limits<double>::max();
    is_able_to_return_to_current_lane = false;
    is_stuck = false;
    is_abort = false;
  }
};
}  // namespace autoware::behavior_path_planner::lane_change

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DEBUG_STRUCTS_HPP_
