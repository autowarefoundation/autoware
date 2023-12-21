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

#ifndef BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_
#define BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_

#include "behavior_path_goal_planner_module/goal_searcher_base.hpp"

#include <lane_departure_checker/lane_departure_checker.hpp>

#include "visualization_msgs/msg/detail/marker_array__struct.hpp"
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner::goal_planner_utils
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

lanelet::ConstLanelets getPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance);
PredictedObjects filterObjectsByLateralDistance(
  const Pose & ego_pose, const double vehicle_width, const PredictedObjects & objects,
  const double distance_thresh, const bool filter_inside);

bool isAllowedGoalModification(const std::shared_ptr<RouteHandler> & route_handler);
bool checkOriginalGoalIsInShoulder(const std::shared_ptr<RouteHandler> & route_handler);

// debug
MarkerArray createPullOverAreaMarkerArray(
  const tier4_autoware_utils::MultiPolygon2d area_polygons, const std_msgs::msg::Header & header,
  const std_msgs::msg::ColorRGBA & color, const double z);
MarkerArray createPosesMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color);
MarkerArray createTextsMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color);
MarkerArray createGoalCandidatesMarkerArray(
  const GoalCandidates & goal_candidates, const std_msgs::msg::ColorRGBA & color);
MarkerArray createNumObjectsToAvoidTextsMarkerArray(
  const GoalCandidates & goal_candidates, std::string && ns,
  const std_msgs::msg::ColorRGBA & color);
}  // namespace behavior_path_planner::goal_planner_utils

#endif  // BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_
