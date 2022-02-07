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

#include "behavior_path_planner/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <string>
#include <vector>

namespace marker_utils
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::AvoidPoint;
using behavior_path_planner::AvoidPointArray;
using behavior_path_planner::ShiftPoint;
using behavior_path_planner::ShiftPointArray;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::Pose;
using visualization_msgs::msg::MarkerArray;

MarkerArray createShiftLengthMarkerArray(
  const std::vector<double> shift_distance, const PathWithLaneId & reference,
  const std::string & ns, const double r, const double g, const double b);

MarkerArray createAvoidPointMarkerArray(
  const AvoidPointArray & shift_points, const std::string & ns, const double r, const double g,
  const double b, const double w);

MarkerArray createShiftPointMarkerArray(
  const ShiftPointArray & shift_points, const double base_shift, const std::string & ns,
  const double r, const double g, const double b, const double w);

MarkerArray createLaneletsAreaMarkerArray(
  const std::vector<lanelet::ConstLanelet> & lanelets, const std::string & ns, const double r,
  const double g, const double b);

MarkerArray createLaneletPolygonsMarkerArray(
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const std::string & ns,
  const int64_t lane_id);

MarkerArray createPolygonMarkerArray(
  const Polygon & polygon, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b);

MarkerArray createObjectsMarkerArray(
  const PredictedObjects & objects, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b);

MarkerArray createAvoidanceObjectsMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects, const std::string & ns);

MarkerArray createPathMarkerArray(
  const PathWithLaneId & path, const std::string & ns, const int64_t lane_id, const double r,
  const double g, const double b);

MarkerArray createVirtualWallMarkerArray(
  const Pose & pose, const int64_t lane_id, const std::string & stop_factor);

MarkerArray createPoseLineMarkerArray(
  const Pose & pose, const std::string & ns, const int64_t id, const double r, const double g,
  const double b);

MarkerArray createPoseMarkerArray(
  const Pose & pose, const std::string & ns, const int64_t id, const double r, const double g,
  const double b);

MarkerArray makeOverhangToRoadShoulderMarkerArray(
  const behavior_path_planner::ObjectDataArray & objects);

MarkerArray createOvehangFurthestLineStringMarkerArray(
  const lanelet::ConstLineStrings3d & linestrings, const std::string & ns, const double r,
  const double g, const double b);
}  // namespace marker_utils

std::string toStrInfo(const behavior_path_planner::ShiftPointArray & sp_arr);

std::string toStrInfo(const behavior_path_planner::AvoidPointArray & ap_arr);

std::string toStrInfo(const behavior_path_planner::ShiftPoint & sp);

std::string toStrInfo(const behavior_path_planner::AvoidPoint & ap);

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__DEBUG_HPP_
