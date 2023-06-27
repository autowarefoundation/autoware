// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECK_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECK_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/marker_util/debug_utilities.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_module_data.hpp"

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <utility>
#include <vector>

namespace behavior_path_planner::utils::safety_check
{

using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebug;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

namespace bg = boost::geometry;
struct ProjectedDistancePoint
{
  Point2d projected_point;
  double distance{0.0};
};

bool isTargetObjectFront(
  const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose,
  const vehicle_info_util::VehicleInfo & vehicle_info, const Polygon2d & obj_polygon);

Polygon2d createExtendedPolygon(
  const Pose & base_link_pose, const vehicle_info_util::VehicleInfo & vehicle_info,
  const double lon_length, const double lat_margin, CollisionCheckDebug & debug);
Polygon2d createExtendedPolygon(
  const Pose & obj_pose, const Shape & shape, const double lon_length, const double lat_margin,
  CollisionCheckDebug & debug);

double calcRssDistance(
  const double front_object_velocity, const double rear_object_velocity,
  const double front_object_deceleration, const double rear_object_deceleration,
  const BehaviorPathPlannerParameters & params);

double calcMinimumLongitudinalLength(
  const double front_object_velocity, const double rear_object_velocity,
  const BehaviorPathPlannerParameters & params);

/**
 * @brief Iterate the points in the ego and target's predicted path and
 *        perform safety check for each of the iterated points.
 * @param planned_path The predicted path of the ego vehicle.
 * @param interpolated_ego A vector of pairs of ego vehicle's pose and its polygon at each moment in
 * the future.
 * @param ego_current_velocity Current velocity of the ego vehicle.
 * @param check_duration The vector of times in the future at which safety check is
 * performed.(relative time in sec from the current time)
 * @param target_object The predicted object to check collision with.
 * @param target_object_path The predicted path of the target object.
 * @param common_parameters The common parameters used in behavior path planner.
 * @param front_object_deceleration The deceleration of the object in the front.(used in RSS)
 * @param rear_object_deceleration The deceleration of the object in the rear.(used in RSS)
 * @param debug The debug information for collision checking.
 * @param prepare_duration The duration to prepare before shifting lane.
 * @param velocity_threshold_for_prepare_duration The threshold for the target velocity to
 * ignore during preparation phase.
 * @return true if distance is safe.
 */
bool isSafeInLaneletCollisionCheck(
  const PathWithLaneId & planned_path,
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & predicted_ego_poses,
  const double ego_current_velocity, const std::vector<double> & check_duration,
  const PredictedObject & target_object, const PredictedPath & target_object_path,
  const BehaviorPathPlannerParameters & common_parameters, const double front_object_deceleration,
  const double rear_object_deceleration, CollisionCheckDebug & debug,
  const double prepare_duration = 0.0, const double velocity_threshold_for_prepare_duration = 0.0);

/**
 * @brief Iterate the points in the ego and target's predicted path and
 *        perform safety check for each of the iterated points.
 * @return true if distance is safe.
 */
bool isSafeInFreeSpaceCollisionCheck(
  const PathWithLaneId & path,
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & interpolated_ego,
  const Twist & ego_current_twist, const std::vector<double> & check_duration,
  const double prepare_duration, const PredictedObject & target_object,
  const BehaviorPathPlannerParameters & common_parameters,
  const double prepare_phase_ignore_target_velocity_thresh, const double front_object_deceleration,
  const double rear_object_deceleration, CollisionCheckDebug & debug);

}  // namespace behavior_path_planner::utils::safety_check

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECK_HPP_
