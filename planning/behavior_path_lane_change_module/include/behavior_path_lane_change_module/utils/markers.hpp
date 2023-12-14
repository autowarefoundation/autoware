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

#ifndef BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__MARKERS_HPP_
#define BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__MARKERS_HPP_

#include "behavior_path_lane_change_module/utils/path.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace marker_utils::lane_change_markers
{
using behavior_path_planner::LaneChangePath;
using behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObjects;
using visualization_msgs::msg::MarkerArray;
MarkerArray showAllValidLaneChangePath(
  const std::vector<LaneChangePath> & lanes, std::string && ns);
MarkerArray createLaneChangingVirtualWallMarker(
  const geometry_msgs::msg::Pose & lane_changing_pose, const std::string & module_name,
  const rclcpp::Time & now, const std::string & ns);
MarkerArray showFilteredObjects(
  const ExtendedPredictedObjects & current_lane_objects,
  const ExtendedPredictedObjects & target_lane_objects,
  const ExtendedPredictedObjects & other_lane_objects, const std::string & ns);
}  // namespace marker_utils::lane_change_markers
#endif  // BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__MARKERS_HPP_
