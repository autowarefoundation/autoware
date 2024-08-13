// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__MARKERS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__MARKERS_HPP_

#include "autoware/behavior_path_lane_change_module/utils/debug_structs.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <geometry_msgs/msg/detail/polygon__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace marker_utils::lane_change_markers
{
using autoware::behavior_path_planner::FilteredByLanesExtendedObjects;
using autoware::behavior_path_planner::LaneChangePath;
using autoware::behavior_path_planner::lane_change::Debug;
using autoware::behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObjects;
using visualization_msgs::msg::MarkerArray;
MarkerArray showAllValidLaneChangePath(
  const std::vector<LaneChangePath> & lanes, std::string && ns);
MarkerArray createLaneChangingVirtualWallMarker(
  const geometry_msgs::msg::Pose & lane_changing_pose, const std::string & module_name,
  const rclcpp::Time & now, const std::string & ns);
MarkerArray showFilteredObjects(
  const FilteredByLanesExtendedObjects & filtered_objects, const std::string & ns);
MarkerArray createExecutionArea(const geometry_msgs::msg::Polygon & execution_area);
MarkerArray showExecutionInfo(const Debug & debug_data, const geometry_msgs::msg::Pose & ego_pose);
MarkerArray createDebugMarkerArray(
  const Debug & debug_data, const geometry_msgs::msg::Pose & ego_pose);

}  // namespace marker_utils::lane_change_markers
#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__MARKERS_HPP_
