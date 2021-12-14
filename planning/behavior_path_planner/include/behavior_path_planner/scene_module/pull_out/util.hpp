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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__UTIL_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__UTIL_HPP_

#include "behavior_path_planner/scene_module/pull_out/pull_out_module.hpp"
#include "behavior_path_planner/utilities.hpp"

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
namespace pull_out_utils
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;

PathWithLaneId combineReferencePath(const PathWithLaneId path1, const PathWithLaneId path2);
std::vector<PullOutPath> getPullOutPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets, const Pose & pose,
  const BehaviorPathPlannerParameters & common_parameter,
  const behavior_path_planner::PullOutParameters & parameter, const bool is_retreat_path = false);

PullOutPath getBackPaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanelets,
  const Pose & pose, const BehaviorPathPlannerParameters & common_parameter,
  const behavior_path_planner::PullOutParameters & parameter, const double back_distance);

bool isPathInLanelets4pullover(
  const PathWithLaneId & path, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets);

Pose getBackedPose(
  const Pose & current_pose, const double & yaw_shoulder_lane, const double & back_distance);

std::vector<PullOutPath> selectValidPaths(
  const std::vector<PullOutPath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const lanelet::routing::RoutingGraphContainer & overall_graphs, const Pose & current_pose,
  const bool isInGoalRouteSection, const Pose & goal_pose);
bool selectSafePath(
  const std::vector<PullOutPath> & paths, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const PredictedObjects::ConstSharedPtr & dynamic_objects, const Pose & current_pose,
  const Twist & current_twist, const double vehicle_width,
  const behavior_path_planner::PullOutParameters & ros_parameters,
  const tier4_autoware_utils::LinearRing2d & vehicle_footprint, PullOutPath * selected_path);
bool isPullOutPathSafe(
  const behavior_path_planner::PullOutPath & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes,
  const PredictedObjects::ConstSharedPtr & dynamic_objects,
  const behavior_path_planner::PullOutParameters & ros_parameters,
  const tier4_autoware_utils::LinearRing2d & vehicle_footprint, const bool use_buffer = true,
  const bool use_dynamic_object = false);
bool hasEnoughDistance(
  const PullOutPath & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Pose & current_pose,
  const bool isInGoalRouteSection, const Pose & goal_pose,
  const lanelet::routing::RoutingGraphContainer & overall_graphs);
bool isObjectFront(const Pose & ego_pose, const Pose & obj_pose);
}  // namespace pull_out_utils
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OUT__UTIL_HPP_
