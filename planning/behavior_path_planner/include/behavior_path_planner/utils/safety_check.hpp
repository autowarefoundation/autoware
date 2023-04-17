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

/**
 * @brief Project nearest point on a line segment.
 * @param [in] reference_point point to project
 * @param [in] line segment
 * @return nearest point on the line segment
 */
template <typename Pythagoras = bg::strategy::distance::pythagoras<>>
ProjectedDistancePoint pointToSegment(
  const Point2d & reference_point, const Point2d & polygon_segment_start,
  const Point2d & polygon_segment_end);

/**
 * @brief Find nearest points between two polygon.
 */
void getProjectedDistancePointFromPolygons(
  const Polygon2d & ego_polygon, const Polygon2d & object_polygon, Pose & point_on_ego,
  Pose & point_on_object);

/**
 * @brief get relative pose with reference to the target object.
 * @param [in] absolute pose desired_pose reference pose
 * @param [in] absolute pose target_pose target pose to check
 * @return relative pose of the target
 */
Pose projectCurrentPoseToTarget(const Pose & reference_pose, const Pose & target_pose);

/**
 * @brief find which vehicle is front and rear and check for lateral,
 *        longitudinal physical and longitudinal expected stopping distance between two points
 * @param [in] expected_ego_pose ego vehicle's pose
 * @param [in] ego_current_twist ego vehicle's twist
 * @param [in] expected_object_pose object vehicle's pose
 * @param [in] object_current_twist object vehicle's twist
 * @param [in] param common behavior path planner parameters
 * @param [in] front_decel expected deceleration of front vehicle
 * @param [in] rear_decel expected deceleration of rear vehicle
 * @param [in] debug debug data
 * @return true if distance is safe.
 */
bool hasEnoughDistance(
  const Pose & expected_ego_pose, const Twist & ego_current_twist,
  const Pose & expected_object_pose, const Twist & object_current_twist,
  const BehaviorPathPlannerParameters & param, const double front_decel, const double rear_decel,
  CollisionCheckDebug & debug);

/**
 * @brief Iterate the points in the ego and target's predicted path and
 *        perform safety check for each of the iterated points.
 * @return true if distance is safe.
 */
bool isSafeInLaneletCollisionCheck(
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & interpolated_ego,
  const Twist & ego_current_twist, const std::vector<double> & check_duration,
  const double prepare_duration, const PredictedObject & target_object,
  const PredictedPath & target_object_path, const BehaviorPathPlannerParameters & common_parameters,
  const double prepare_phase_ignore_target_speed_thresh, const double front_decel,
  const double rear_decel, Pose & ego_pose_before_collision, CollisionCheckDebug & debug);

/**
 * @brief Iterate the points in the ego and target's predicted path and
 *        perform safety check for each of the iterated points.
 * @return true if distance is safe.
 */
bool isSafeInFreeSpaceCollisionCheck(
  const std::vector<std::pair<Pose, tier4_autoware_utils::Polygon2d>> & interpolated_ego,
  const Twist & ego_current_twist, const std::vector<double> & check_duration,
  const double prepare_duration, const PredictedObject & target_object,
  const BehaviorPathPlannerParameters & common_parameters,
  const double prepare_phase_ignore_target_speed_thresh, const double front_decel,
  const double rear_decel, CollisionCheckDebug & debug);

}  // namespace behavior_path_planner::utils::safety_check

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__SAFETY_CHECK_HPP_
