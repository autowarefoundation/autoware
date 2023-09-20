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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__OBJECTS_FILTERING_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__OBJECTS_FILTERING_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner::utils::path_safety_checker
{

using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;

/**
 * @brief Filters objects based on various criteria.
 *
 * @param objects The predicted objects to filter.
 * @param route_handler
 * @param current_lanes
 * @param current_pose The current pose of ego vehicle.
 * @param params The filtering parameters.
 * @return PredictedObjects The filtered objects.
 */
PredictedObjects filterObjects(
  const std::shared_ptr<const PredictedObjects> & objects,
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelets & current_lanes,
  const geometry_msgs::msg::Point & current_pose,
  const std::shared_ptr<ObjectsFilteringParams> & params);

/**
 * @brief Filters objects based on their velocity.
 *
 * Depending on the remove_above_threshold parameter, this function either removes objects with
 * velocities above the given threshold or only keeps those objects. It uses the helper function
 * filterObjectsByVelocity() to do the actual filtering.
 *
 * @param objects The objects to be filtered.
 * @param velocity_threshold The velocity threshold for the filtering.
 * @param remove_above_threshold If true, objects with velocities above the threshold are removed.
 *                               If false, only objects with velocities above the threshold are
 * kept.
 * @return A new collection of objects that have been filtered according to the rules.
 */
PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, const double velocity_threshold,
  const bool remove_above_threshold = true);

/**
 * @brief Helper function to filter objects based on their velocity.
 *
 * This function iterates over all objects and calculates their velocity norm. If the velocity norm
 * is within the velocity_threshold and max_velocity range, the object is added to a new collection.
 * This new collection is then returned.
 *
 * @param objects The objects to be filtered.
 * @param velocity_threshold The minimum velocity for an object to be included in the output.
 * @param max_velocity The maximum velocity for an object to be included in the output.
 * @return A new collection of objects that have been filtered according to the rules.
 */
PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, double velocity_threshold, double max_velocity);

/**
 * @brief Filter objects based on their position relative to a current_pose.
 *
 * @param objects The predicted objects to filter.
 * @param path_points Points on the path.
 * @param current_pose Current pose of the reference (e.g., ego vehicle).
 * @param forward_distance Maximum forward distance for filtering.
 * @param backward_distance Maximum backward distance for filtering.
 */
void filterObjectsByPosition(
  PredictedObjects & objects, const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Point & current_pose, const double forward_distance,
  const double backward_distance);

/**
 * @brief Filters the provided objects based on their classification.
 *
 * @param objects The predicted objects to be filtered.
 * @param target_object_types The types of objects to retain after filtering.
 */
void filterObjectsByClass(
  PredictedObjects & objects, const ObjectTypesToCheck & target_object_types);

/**
 * @brief Separate index of the obstacles into two part based on whether the object is within
 * lanelet.
 * @return Indices of objects pair. first objects are in the lanelet, and second others are out of
 * lanelet.
 */
std::pair<std::vector<size_t>, std::vector<size_t>> separateObjectIndicesByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets);

/**
 * @brief Separate the objects into two part based on whether the object is within lanelet.
 * @return Objects pair. first objects are in the lanelet, and second others are out of lanelet.
 */
std::pair<PredictedObjects, PredictedObjects> separateObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets);

/**
 * @brief Get the predicted path from an object.
 *
 * @param obj The extended predicted object.
 * @param is_use_all_predicted_path Flag to determine whether to use all predicted paths or just the
 * one with maximum confidence.
 * @return std::vector<PredictedPathWithPolygon> The predicted path(s) from the object.
 */
std::vector<PredictedPathWithPolygon> getPredictedPathFromObj(
  const ExtendedPredictedObject & obj, const bool & is_use_all_predicted_path);

/**
 * @brief Create a predicted path using the provided parameters.
 *
 * The function predicts the path based on the current vehicle pose, its current velocity,
 * and certain parameters related to the vehicle's behavior and environment. The prediction
 * considers acceleration, delay before departure, and maximum velocity constraints.
 *
 * During the delay before departure, the vehicle's velocity is assumed to be zero, and it does
 * not move. After the delay, the vehicle starts to accelerate as per the provided parameters
 * until it reaches the maximum allowable velocity or the specified time horizon.
 *
 * @param ego_predicted_path_params Parameters associated with the ego's predicted path behavior.
 * @param path_points Path points to be followed by the vehicle.
 * @param vehicle_pose Current pose of the vehicle.
 * @param current_velocity Current velocity of the vehicle.
 * @param ego_seg_idx Segment index where the ego vehicle is currently located on the path.
 * @param is_object_front Flag indicating if there is an object in front of the ego vehicle.
 * @param limit_to_max_velocity Flag indicating if the predicted path should consider the
 *                              maximum allowable velocity.
 * @return std::vector<PoseWithVelocityStamped> Predicted path based on the input parameters.
 */
std::vector<PoseWithVelocityStamped> createPredictedPath(
  const std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::vector<PathPointWithLaneId> & path_points,
  const geometry_msgs::msg::Pose & vehicle_pose, const double current_velocity,
  const size_t ego_seg_idx, const bool is_object_front, const bool limit_to_max_velocity);

/**
 * @brief Checks if the centroid of a given object is within the provided lanelets.
 *
 * @param object The predicted object to check.
 * @param target_lanelets The lanelets to check against.
 * @return bool True if the object's centroid is within the lanelets, otherwise false.
 */
bool isCentroidWithinLanelets(
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets);

/**
 * @brief Transforms a given object into an extended predicted object.
 *
 * @param object The predicted object to transform.
 * @param safety_check_time_horizon The time horizon for safety checks.
 * @param safety_check_time_resolution The time resolution for safety checks.
 * @return ExtendedPredictedObject The transformed object.
 */
ExtendedPredictedObject transform(
  const PredictedObject & object, const double safety_check_time_horizon,
  const double safety_check_time_resolution);

/**
 * @brief Creates target objects on a lane based on provided parameters.
 *
 * @param current_lanes
 * @param route_handler
 * @param filtered_objects The filtered objects.
 * @param params The filtering parameters.
 * @return TargetObjectsOnLane The target objects on the lane.
 */
TargetObjectsOnLane createTargetObjectsOnLane(
  const lanelet::ConstLanelets & current_lanes, const std::shared_ptr<RouteHandler> & route_handler,
  const PredictedObjects & filtered_objects,
  const std::shared_ptr<ObjectsFilteringParams> & params);

/**
 * @brief Determines whether the predicted object type matches any of the target object types
 * specified by the user.
 *
 * @param object The predicted object whose type is to be checked.
 * @param target_object_types A structure containing boolean flags for each object type that the
 * user is interested in checking.
 *
 * @return Returns true if the predicted object's highest probability label matches any of the
 * specified target object types.
 */
bool isTargetObjectType(
  const PredictedObject & object, const ObjectTypesToCheck & target_object_types);

}  // namespace behavior_path_planner::utils::path_safety_checker

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__OBJECTS_FILTERING_HPP_
