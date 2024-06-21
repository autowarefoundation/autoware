// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__UTILS_HPP_

#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_planner_common/parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "rclcpp/logger.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::utils::lane_change
{
using autoware::behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
using autoware::behavior_path_planner::utils::path_safety_checker::
  PoseWithVelocityAndPolygonStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::PredictedPathWithPolygon;
using autoware::route_handler::Direction;
using autoware::universe_utils::Polygon2d;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using behavior_path_planner::lane_change::LanesPolygon;
using behavior_path_planner::lane_change::PathSafetyStatus;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using path_safety_checker::CollisionCheckDebugMap;
using tier4_planning_msgs::msg::PathWithLaneId;

double calcLaneChangeResampleInterval(
  const double lane_changing_length, const double lane_changing_velocity);

double calcMinimumLaneChangeLength(
  const LaneChangeParameters & lane_change_parameters, const std::vector<double> & shift_intervals);

double calcMinimumLaneChangeLength(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelet & lane,
  const LaneChangeParameters & lane_change_parameters, Direction direction);

double calcMaximumLaneChangeLength(
  const double current_velocity, const LaneChangeParameters & lane_change_parameters,
  const std::vector<double> & shift_intervals, const double max_acc);

double calcMinimumAcceleration(
  const double current_velocity, const double min_longitudinal_acc,
  const LaneChangeParameters & lane_change_parameters);

double calcMaximumAcceleration(
  const double current_velocity, const double current_max_velocity,
  const double max_longitudinal_acc, const LaneChangeParameters & lane_change_parameters);

double calcLaneChangingAcceleration(
  const double initial_lane_changing_velocity, const double max_path_velocity,
  const double lane_changing_time, const double prepare_longitudinal_acc);

void setPrepareVelocity(
  PathWithLaneId & prepare_segment, const double current_velocity, const double prepare_velocity);

std::vector<double> getAccelerationValues(
  const double min_acc, const double max_acc, const size_t sampling_num);

std::vector<int64_t> replaceWithSortedIds(
  const std::vector<int64_t> & original_lane_ids,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids);

std::vector<std::vector<int64_t>> getSortedLaneIds(
  const RouteHandler & route_handler, const Pose & current_pose,
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes);

lanelet::ConstLanelets getTargetPreferredLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Direction & direction,
  const LaneChangeModuleType & type);

lanelet::ConstLanelets getTargetNeighborLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const LaneChangeModuleType & type);

lanelet::BasicPolygon2d getTargetNeighborLanesPolygon(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType & type);

bool isPathInLanelets(
  const PathWithLaneId & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes);

std::optional<LaneChangePath> constructCandidatePath(
  const LaneChangeInfo & lane_change_info, const PathWithLaneId & prepare_segment,
  const PathWithLaneId & target_segment, const PathWithLaneId & target_lane_reference_path,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids);

ShiftLine getLaneChangingShiftLine(
  const PathWithLaneId & prepare_segment, const PathWithLaneId & target_segment,
  const PathWithLaneId & reference_path, const double shift_length);

ShiftLine getLaneChangingShiftLine(
  const Pose & lane_changing_start_pose, const Pose & lane_changing_end_pose,
  const PathWithLaneId & reference_path, const double shift_length);

PathWithLaneId getReferencePathFromTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & lane_changing_start_pose, const double target_lane_length,
  const double lane_changing_length, const double forward_path_length,
  const double resample_interval, const bool is_goal_in_route,
  const double next_lane_change_buffer);

std::vector<DrivableLanes> generateDrivableLanes(
  const std::vector<DrivableLanes> original_drivable_lanes, const RouteHandler & route_handler,
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & lane_change_lanes);

std::vector<DrivableLanes> generateDrivableLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & lane_change_lanes);

double getLateralShift(const LaneChangePath & path);

bool hasEnoughLengthToLaneChangeAfterAbort(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelets & current_lanes,
  const Pose & curent_pose, const double abort_return_dist,
  const LaneChangeParameters & lane_change_parameters, const Direction direction);

double calcLateralBufferForFiltering(const double vehicle_width, const double lateral_buffer = 0.0);

double calcLateralBufferForFiltering(const double vehicle_width, const double lateral_buffer);

std::string getStrDirection(const std::string & name, const Direction direction);

CandidateOutput assignToCandidate(
  const LaneChangePath & lane_change_path, const Point & ego_position);
std::optional<lanelet::ConstLanelet> getLaneChangeTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType type, const Direction & direction);

std::vector<PoseWithVelocityStamped> convertToPredictedPath(
  const LaneChangePath & lane_change_path, const Twist & vehicle_twist, const Pose & pose,
  const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters, const double resolution);

bool isParkedObject(
  const PathWithLaneId & path, const RouteHandler & route_handler,
  const ExtendedPredictedObject & object, const double object_check_min_road_shoulder_width,
  const double object_shiftable_ratio_threshold,
  const double static_object_velocity_threshold = 1.0);

bool isParkedObject(
  const lanelet::ConstLanelet & closest_lanelet, const lanelet::BasicLineString2d & boundary,
  const ExtendedPredictedObject & object, const double buffer_to_bound,
  const double ratio_threshold);

bool passParkedObject(
  const RouteHandler & route_handler, const LaneChangePath & lane_change_path,
  const std::vector<ExtendedPredictedObject> & objects, const double minimum_lane_change_length,
  const bool is_goal_in_route, const LaneChangeParameters & lane_change_parameters,
  CollisionCheckDebugMap & object_debug);

std::optional<size_t> getLeadingStaticObjectIdx(
  const RouteHandler & route_handler, const LaneChangePath & lane_change_path,
  const std::vector<ExtendedPredictedObject> & objects,
  const double object_check_min_road_shoulder_width, const double object_shiftable_ratio_threshold);

std::optional<lanelet::BasicPolygon2d> createPolygon(
  const lanelet::ConstLanelets & lanes, const double start_dist, const double end_dist);

ExtendedPredictedObject transform(
  const PredictedObject & object, const BehaviorPathPlannerParameters & common_parameters,
  const LaneChangeParameters & lane_change_parameters, const bool check_at_prepare_phase);

bool isCollidedPolygonsInLanelet(
  const std::vector<Polygon2d> & collided_polygons, const lanelet::ConstLanelets & lanes);

/**
 * @brief Generates expanded lanelets based on the given direction and offsets.
 *
 * Expands the provided lanelets in either the left or right direction based on
 * the specified direction. If the direction is 'LEFT', the lanelets are expanded
 * using the left_offset; if 'RIGHT', they are expanded using the right_offset.
 * Otherwise, no expansion occurs.
 *
 * @param lanes The lanelets to be expanded.
 * @param direction The direction of expansion: either LEFT or RIGHT.
 * @param left_offset The offset value for left expansion.
 * @param right_offset The offset value for right expansion.
 * @return lanelet::ConstLanelets A collection of expanded lanelets.
 */
lanelet::ConstLanelets generateExpandedLanelets(
  const lanelet::ConstLanelets & lanes, const Direction direction, const double left_offset,
  const double right_offset);

/**
 * @brief Retrieves a logger instance for a specific lane change type.
 *
 * This function provides a specialized logger for different types of lane change.
 *
 * @param type A string representing the type of lane change operation. This could be
 *             a specific maneuver or condition related to lane changing, such as
 *             'avoidance_by_lane_change', 'normal', 'external_request'.
 *
 * @return rclcpp::Logger The logger instance configured for the specified lane change type.
 */
rclcpp::Logger getLogger(const std::string & type);

/**
 * @brief Computes the current footprint of the ego vehicle based on its pose and size.
 *
 * This function calculates the 2D polygon representing the current footprint of the ego vehicle.
 * The footprint is determined by the vehicle's pose and its dimensions, including the distance
 * from the base to the front and rear ends of the vehicle, as well as its width.
 *
 * @param ego_pose The current pose of the ego vehicle.
 * @param ego_info The structural information of the ego vehicle, such as its maximum longitudinal
 *                 offset, rear overhang, and width.
 *
 * @return Polygon2d A polygon representing the current 2D footprint of the ego vehicle.
 */
Polygon2d getEgoCurrentFootprint(
  const Pose & ego_pose, const autoware::vehicle_info_utils::VehicleInfo & ego_info);

/**
 * @brief Checks if the given polygon is within an intersection area.
 *
 * This function evaluates whether a specified polygon is located within the bounds of an
 * intersection. It identifies the intersection area by checking the attributes of the provided
 * lanelet. If the lanelet has an attribute indicating it is part of an intersection, the function
 * then checks if the polygon is fully contained within this area.
 *
 * @param route_handler a shared pointer to the route_handler
 * @param lanelet A lanelet to check against the
 *                intersection area.
 * @param polygon The polygon to check for containment within the intersection area.
 *
 * @return bool True if the polygon is within the intersection area, false otherwise.
 */
bool isWithinIntersection(
  const std::shared_ptr<RouteHandler> & route_handler, const lanelet::ConstLanelet & lanelet,
  const Polygon2d & polygon);

/**
 * @brief Determines if a polygon is within lanes designated for turning.
 *
 * Checks if a polygon overlaps with lanelets tagged for turning directions (excluding 'straight').
 * It evaluates the lanelet's 'turn_direction' attribute and determines overlap with the lanelet's
 * area.
 *
 * @param lanelet Lanelet representing the road segment whose turn direction is to be evaluated.
 * @param polygon The polygon to be checked for its presence within turn direction lanes.
 *
 * @return bool True if the polygon is within a lane designated for turning, false if it is within a
 *              straight lane or no turn direction is specified.
 */
bool isWithinTurnDirectionLanes(const lanelet::ConstLanelet & lanelet, const Polygon2d & polygon);

/**
 * @brief Calculates the distance required during a lane change operation.
 *
 * Used for computing prepare or lane change length based on current and maximum velocity,
 * acceleration, and duration, returning the lesser of accelerated distance or distance at max
 * velocity.
 *
 * @param velocity The current velocity of the vehicle in meters per second (m/s).
 * @param maximum_velocity The maximum velocity the vehicle can reach in meters per second (m/s).
 * @param acceleration The acceleration of the vehicle in meters per second squared (m/s^2).
 * @param duration The duration of the lane change in seconds (s).
 * @return The calculated minimum distance in meters (m).
 */
double calcPhaseLength(
  const double velocity, const double maximum_velocity, const double acceleration,
  const double time);

LanesPolygon createLanesPolygon(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const std::vector<lanelet::ConstLanelets> & target_backward_lanes);
}  // namespace autoware::behavior_path_planner::utils::lane_change

namespace autoware::behavior_path_planner::utils::lane_change::debug
{
geometry_msgs::msg::Point32 create_point32(const geometry_msgs::msg::Pose & pose);

geometry_msgs::msg::Polygon createExecutionArea(
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const Pose & pose,
  double additional_lon_offset, double additional_lat_offset);
}  // namespace autoware::behavior_path_planner::utils::lane_change::debug

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__UTILS_HPP_
