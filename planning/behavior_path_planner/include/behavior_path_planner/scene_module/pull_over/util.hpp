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
lanelet::ConstLanelets getPullOverLanes(const RouteHandler & route_handler);
bool hasEnoughDistanceToParkingStart(
  const PathWithLaneId & path, const Pose & current_pose, const Pose & start_pose,
  const double current_vel, const double maximum_deceleration, const double decide_path_distance,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold);

// debug
Marker createPullOverAreaMarker(
  const Pose & start_pose, const Pose & end_pose, const int32_t id,
  const std_msgs::msg::Header & header, const double base_link2front, const double base_link2rear,
  const double vehicle_width, const std_msgs::msg::ColorRGBA & color);
}  // namespace pull_over_utils
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__UTIL_HPP_
