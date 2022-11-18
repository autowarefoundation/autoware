// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__DEBUG_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__DEBUG_HPP_

#include "behavior_path_planner/debug_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <string>
#include <vector>

namespace marker_utils::avoidance_marker
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::AvoidLineArray;
using behavior_path_planner::ShiftLineArray;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::Pose;
using visualization_msgs::msg::MarkerArray;

MarkerArray createAvoidLineMarkerArray(
  const AvoidLineArray & shift_points, std::string && ns, const float & r, const float & g,
  const float & b, const double & w);

MarkerArray createTargetObjectsMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, std::string && ns);

MarkerArray createOtherObjectsMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, std::string && ns);

MarkerArray makeOverhangToRoadShoulderMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, std::string && ns);

MarkerArray createOverhangFurthestLineStringMarkerArray(
  const lanelet::ConstLineStrings3d & linestrings, std::string && ns, const float & r,
  const float & g, const float & b);

}  // namespace marker_utils::avoidance_marker

std::string toStrInfo(const behavior_path_planner::ShiftLineArray & sp_arr);

std::string toStrInfo(const behavior_path_planner::AvoidLineArray & ap_arr);

std::string toStrInfo(const behavior_path_planner::ShiftLine & sp);

std::string toStrInfo(const behavior_path_planner::AvoidLine & ap);

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__DEBUG_HPP_
