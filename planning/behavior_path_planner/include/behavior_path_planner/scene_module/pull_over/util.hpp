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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__UTIL_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__UTIL_HPP_

#include "behavior_path_planner/scene_module/pull_over/pull_over_module.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lane_departure_checker/lane_departure_checker.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <lanelet2_core/primitives/Primitive.h>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
namespace pull_over_utils
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;

// TODO(sugahara) move to util
PathWithLaneId combineReferencePath(const PathWithLaneId path1, const PathWithLaneId path2);

std::vector<ShiftParkingPath> generateShiftParkingPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets, const Pose & pose, const Pose & goal_pose,
  const BehaviorPathPlannerParameters & common_parameter,
  const behavior_path_planner::PullOverParameters & parameter);

std::vector<ShiftParkingPath> selectValidPaths(
  const std::vector<ShiftParkingPath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Pose & current_pose,
  const bool is_in_goal_route_section, const Pose & goal_pose,
  const lane_departure_checker::LaneDepartureChecker & lane_departure_checker);
bool selectSafePath(
  const std::vector<ShiftParkingPath> & paths,
  const OccupancyGridBasedCollisionDetector & occupancy_grid_map, ShiftParkingPath & selected_path);
bool hasEnoughDistance(
  const ShiftParkingPath & path, const lanelet::ConstLanelets & current_lanes,
  const Pose & current_pose, const bool is_in_goal_route_section, const Pose & goal_pose);
lanelet::ConstLanelets getPullOverLanes(const RouteHandler & route_handler);

// debug
Marker createPullOverAreaMarker(
  const Pose & start_pose, const Pose & end_pose, const int32_t id,
  const std_msgs::msg::Header & header, const double base_link2front, const double base_link2rear,
  const double vehicle_width, const std_msgs::msg::ColorRGBA & color);
}  // namespace pull_over_utils
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__UTIL_HPP_
