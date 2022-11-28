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

#ifndef BEHAVIOR_PATH_PLANNER__UTILITIES_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILITIES_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/debug_utilities.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_path.hpp"

#include <opencv2/opencv.hpp>
#include <route_handler/route_handler.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_path.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner::util
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;

using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::OccupancyGrid;
using route_handler::RouteHandler;
using tier4_autoware_utils::LineString2d;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
namespace bg = boost::geometry;
using geometry_msgs::msg::Pose;
using marker_utils::CollisionCheckDebug;
using vehicle_info_util::VehicleInfo;

struct FrenetCoordinate3d
{
  double length{0.0};    // longitudinal
  double distance{0.0};  // lateral
};

struct ProjectedDistancePoint
{
  Point2d projected_point;
  double distance{0.0};
};

template <typename Pythagoras = bg::strategy::distance::pythagoras<> >
ProjectedDistancePoint pointToSegment(
  const Point2d & reference_point, const Point2d & point_from_ego,
  const Point2d & point_from_object);

void getProjectedDistancePointFromPolygons(
  const Polygon2d & ego_polygon, const Polygon2d & object_polygon, Pose & point_on_ego,
  Pose & point_on_object);
// data conversions

std::vector<Pose> convertToPoseArray(const PathWithLaneId & path);

std::vector<Point> convertToGeometryPointArray(const PathWithLaneId & path);

PoseArray convertToGeometryPoseArray(const PathWithLaneId & path);

PredictedPath convertToPredictedPath(
  const PathWithLaneId & path, const Twist & vehicle_twist, const Pose & pose,
  const double nearest_seg_idx, const double duration, const double resolution,
  const double acceleration, const double min_speed = 1.0);

template <class T>
FrenetCoordinate3d convertToFrenetCoordinate3d(
  const std::vector<T> & pose_array, const Point & search_point_geom, const size_t seg_idx)
{
  FrenetCoordinate3d frenet_coordinate;

  const double longitudinal_length =
    motion_utils::calcLongitudinalOffsetToSegment(pose_array, seg_idx, search_point_geom);
  frenet_coordinate.length =
    motion_utils::calcSignedArcLength(pose_array, 0, seg_idx) + longitudinal_length;
  frenet_coordinate.distance =
    motion_utils::calcLateralOffset(pose_array, search_point_geom, seg_idx);

  return frenet_coordinate;
}

inline FrenetCoordinate3d convertToFrenetCoordinate3d(
  const PathWithLaneId & path, const Point & search_point_geom, const size_t seg_idx)
{
  return convertToFrenetCoordinate3d(path.points, search_point_geom, seg_idx);
}

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets);

// distance (arclength) calculation

double l2Norm(const Vector3 vector);

double getDistanceToEndOfLane(const Pose & current_pose, const lanelet::ConstLanelets & lanelets);

double getDistanceToNextIntersection(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets);

double getDistanceToCrosswalk(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphContainer & overall_graphs);

double getSignedDistance(
  const Pose & current_pose, const Pose & goal_pose, const lanelet::ConstLanelets & lanelets);

double getArcLengthToTargetLanelet(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelet & target_lane,
  const Pose & pose);

// object collision check

Pose lerpByPose(const Pose & p1, const Pose & p2, const double t);

inline Point lerpByPoint(const Point & p1, const Point & p2, const double t)
{
  tf2::Vector3 v1, v2;
  v1.setValue(p1.x, p1.y, p1.z);
  v2.setValue(p2.x, p2.y, p2.z);

  const auto lerped_point = v1.lerp(v2, t);

  Point point;
  point.x = lerped_point.x();
  point.y = lerped_point.y();
  point.z = lerped_point.z();
  return point;
}

template <class T>
Point lerpByLength(const std::vector<T> & point_array, const double length)
{
  Point lerped_point;
  if (point_array.empty()) {
    return lerped_point;
  }
  Point prev_geom_pt = tier4_autoware_utils::getPoint(point_array.front());
  double accumulated_length = 0;
  for (const auto & pt : point_array) {
    const auto & geom_pt = tier4_autoware_utils::getPoint(pt);
    const double distance = tier4_autoware_utils::calcDistance3d(prev_geom_pt, geom_pt);
    if (accumulated_length + distance > length) {
      return lerpByPoint(prev_geom_pt, geom_pt, (length - accumulated_length) / distance);
    }
    accumulated_length += distance;
    prev_geom_pt = geom_pt;
  }

  return tier4_autoware_utils::getPoint(point_array.back());
}

bool lerpByTimeStamp(const PredictedPath & path, const double t, Pose * lerped_pt);

bool calcObjectPolygon(const PredictedObject & object, Polygon2d * object_polygon);

bool calcObjectPolygon(
  const Shape & object_shape, const Pose & object_pose, Polygon2d * object_polygon);

PredictedPath resamplePredictedPath(
  const PredictedPath & input_path, const double resolution, const double duration);

double getDistanceBetweenPredictedPaths(
  const PredictedPath & path1, const PredictedPath & path2, const double start_time,
  const double end_time, const double resolution);

double getDistanceBetweenPredictedPathAndObject(
  const PredictedObject & object, const PredictedPath & path, const double start_time,
  const double end_time, const double resolution);

/**
 * @brief Check collision between ego path footprints and objects.
 * @return Has collision or not
 */
bool checkCollisionBetweenPathFootprintsAndObjects(
  const tier4_autoware_utils::LinearRing2d & vehicle_footprint, const PathWithLaneId & ego_path,
  const PredictedObjects & dynamic_objects, const double margin);

/**
 * @brief Check collision between ego footprints and objects.
 * @return Has collision or not
 */
bool checkCollisionBetweenFootprintAndObjects(
  const tier4_autoware_utils::LinearRing2d & vehicle_footprint, const Pose & ego_pose,
  const PredictedObjects & dynamic_objects, const double margin);

/**
 * @brief calculate lateral distance from ego pose to object
 * @return distance from ego pose to object
 */
double calcLateralDistanceFromEgoToObject(
  const Pose & ego_pose, const double vehicle_width, const PredictedObject & dynamic_object);

/**
 * @brief calculate longitudinal distance from ego pose to object
 * @return distance from ego pose to object
 */
double calcLongitudinalDistanceFromEgoToObject(
  const Pose & ego_pose, const double base_link2front, const double base_link2rear,
  const PredictedObject & dynamic_object);

/**
 * @brief calculate minimum longitudinal distance from ego pose to objects
 * @return minimum distance from ego pose to objects
 */
double calcLongitudinalDistanceFromEgoToObjects(
  const Pose & ego_pose, double base_link2front, double base_link2rear,
  const PredictedObjects & dynamic_objects);

/**
 * @brief Get index of the obstacles inside the lanelets with start and end length
 * @return Indices corresponding to the obstacle inside the lanelets
 */
std::vector<size_t> filterObjectIndicesByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & lanelets,
  const double start_arc_length, const double end_arc_length);

/**
 * @brief Get index of the obstacles inside the lanelets
 * @return Indices corresponding to the obstacle inside the lanelets
 */
std::vector<size_t> filterObjectIndicesByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets);

PredictedObjects filterObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets);

std::vector<size_t> filterObjectsIndicesByPath(
  const PredictedObjects & objects, const std::vector<size_t> & object_indices,
  const PathWithLaneId & ego_path, const double vehicle_width);

PredictedObjects filterObjectsByVelocity(const PredictedObjects & objects, double lim_v);

PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, double min_v, double max_v);

// drivable area generation
lanelet::ConstLanelets transformToLanelets(const DrivableLanes & drivable_lanes);
lanelet::ConstLanelets transformToLanelets(const std::vector<DrivableLanes> & drivable_lanes);
boost::optional<lanelet::ConstLanelet> getRightLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes);
boost::optional<lanelet::ConstLanelet> getLeftLanelet(
  const lanelet::ConstLanelet & current_lane, const lanelet::ConstLanelets & shoulder_lanes);
std::vector<DrivableLanes> generateDrivableLanes(const lanelet::ConstLanelets & current_lanes);
std::vector<DrivableLanes> generateDrivableLanesWithShoulderLanes(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & shoulder_lanes);

void occupancyGridToImage(const OccupancyGrid & occupancy_grid, cv::Mat * cv_image);

void imageToOccupancyGrid(const cv::Mat & cv_image, OccupancyGrid * occupancy_grid);

cv::Point toCVPoint(
  const Point & geom_point, const double width_m, const double height_m, const double resolution);

OccupancyGrid generateDrivableArea(
  const PathWithLaneId & path, const std::vector<DrivableLanes> & lanes, const double resolution,
  const double vehicle_length, const std::shared_ptr<const PlannerData> planner_data);

lanelet::ConstLineStrings3d getDrivableAreaForAllSharedLinestringLanelets(
  const std::shared_ptr<const PlannerData> & planner_data);

/**
 * @brief Expand the borders of the given lanelets
 * @param [in] drivable_lanes lanelets to expand
 * @param [in] left_bound_offset [m] expansion distance of the left bound
 * @param [in] right_bound_offset [m] expansion distance of the right bound
 * @param [in] types_to_skip linestring types that will not be expanded
 * @return expanded lanelets
 */
std::vector<DrivableLanes> expandLanelets(
  const std::vector<DrivableLanes> & drivable_lanes, const double left_bound_offset,
  const double right_bound_offset, const std::vector<std::string> & types_to_skip = {});

// goal management

/**
 * @brief Modify the path points near the goal to smoothly connect the input path and the goal
 * point
 * @details Remove the path points that are forward from the goal by the distance of
 * search_radius_range. Then insert the goal into the path. The previous goal point generated
 * from the goal posture information is also inserted for the smooth connection of the goal pose.
 * @param [in] search_radius_range distance on path to be modified for goal insertion
 * @param [in] search_rad_range [unused]
 * @param [in] input original path
 * @param [in] goal original goal pose
 * @param [in] goal_lane_id [unused]
 * @param [in] output_ptr output path with modified points for the goal
 */
bool setGoal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const Pose & goal, const int64_t goal_lane_id, PathWithLaneId * output_ptr);

/**
 * @brief Recreate the goal pose to prevent the goal point being too far from the lanelet, which
 *  causes the path to twist near the goal.
 * @details Return the goal point projected on the straight line of the segment of lanelet
 *  closest to the original goal.
 * @param [in] goal original goal pose
 * @param [in] goal_lanelet lanelet containing the goal pose
 */
const Pose refineGoal(const Pose & goal, const lanelet::ConstLanelet & goal_lanelet);

PathWithLaneId refinePathForGoal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const Pose & goal, const int64_t goal_lane_id);

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path);

bool containsGoal(const lanelet::ConstLanelets & lanes, const lanelet::Id & goal_id);

// path management

// TODO(Horibe) There is a similar function in route_handler. Check.
std::shared_ptr<PathWithLaneId> generateCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data);

PathPointWithLaneId insertStopPoint(double length, PathWithLaneId * path);

double getSignedDistanceFromShoulderLeftBoundary(
  const lanelet::ConstLanelets & shoulder_lanelets, const Pose & pose);
double getSignedDistanceFromRightBoundary(
  const lanelet::ConstLanelets & lanelets, const Pose & pose);

// misc

lanelet::Polygon3d getVehiclePolygon(
  const Pose & vehicle_pose, const double vehicle_width, const double base_link2front);

std::vector<Polygon2d> getTargetLaneletPolygons(
  const lanelet::ConstLanelets & lanelets, const Pose & pose, const double check_length,
  const std::string & target_type);

void shiftPose(Pose * pose, double shift_length);

// route handler
PathWithLaneId getCenterLinePath(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & lanelet_sequence,
  const Pose & pose, const double backward_path_length, const double forward_path_length,
  const BehaviorPathPlannerParameters & parameter, const double optional_length = 0.0);

PathWithLaneId setDecelerationVelocity(
  const RouteHandler & route_handler, const PathWithLaneId & input,
  const lanelet::ConstLanelets & lanelet_sequence, const double lane_change_prepare_duration,
  const double lane_change_buffer);

bool checkLaneIsInIntersection(
  const RouteHandler & route_handler, const PathWithLaneId & ref,
  const lanelet::ConstLanelets & lanelet_sequence, const BehaviorPathPlannerParameters & parameters,
  const int num_lane_change, double & additional_length_to_add);

PathWithLaneId setDecelerationVelocity(
  const PathWithLaneId & input, const double target_velocity, const Pose target_pose,
  const double buffer, const double deceleration_interval);

PathWithLaneId setDecelerationVelocityForTurnSignal(
  const PathWithLaneId & input, const Pose target_pose, const double turn_light_on_threshold_time);

// object label
std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification);

lanelet::ConstLanelets getCurrentLanes(const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets extendLanes(
  const std::shared_ptr<RouteHandler> route_handler, const lanelet::ConstLanelets & lanes);

lanelet::ConstLanelets getExtendedCurrentLanes(
  const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<RouteHandler> route_handler, const geometry_msgs::msg::Pose & pose,
  const double forward_length, const double backward_length);

Polygon2d convertBoundingBoxObjectToGeometryPolygon(
  const Pose & current_pose, const double & base_to_front, const double & base_to_rear,
  const double & base_to_width);

Polygon2d convertCylindricalObjectToGeometryPolygon(
  const Pose & current_pose, const Shape & obj_shape);

Polygon2d convertPolygonObjectToGeometryPolygon(const Pose & current_pose, const Shape & obj_shape);

std::string getUuidStr(const PredictedObject & obj);

std::vector<PredictedPath> getPredictedPathFromObj(
  const PredictedObject & obj, const bool & is_use_all_predicted_path);

Pose projectCurrentPoseToTarget(const Pose & desired_object, const Pose & target_object);

bool getEgoExpectedPoseAndConvertToPolygon(
  const Pose & current_pose, const PredictedPath & pred_path,
  tier4_autoware_utils::Polygon2d & ego_polygon, const double & check_current_time,
  const VehicleInfo & ego_info, Pose & expected_pose, std::string & failed_reason);

bool getObjectExpectedPoseAndConvertToPolygon(
  const PredictedPath & pred_path, const PredictedObject & object, Polygon2d & obj_polygon,
  const double & check_current_time, Pose & expected_pose, std::string & failed_reason);

bool isObjectFront(const Pose & ego_pose, const Pose & obj_pose);

bool isObjectFront(const Pose & projected_ego_pose);

double stoppingDistance(const double & vehicle_velocity, const double & vehicle_accel);

double frontVehicleStopDistance(
  const double & front_vehicle_velocity, const double & front_vehicle_accel,
  const double & distance_to_collision);

double rearVehicleStopDistance(
  const double & rear_vehicle_velocity, const double & rear_vehicle_accel,
  const double & rear_vehicle_reaction_time, const double & rear_vehicle_safety_time_margin);

bool isLongitudinalDistanceEnough(
  const double & rear_vehicle_stop_threshold, const double & front_vehicle_stop_threshold);

bool hasEnoughDistance(
  const Pose & expected_ego_pose, const Twist & ego_current_twist,
  const Pose & expected_object_pose, const Twist & object_current_twist,
  const BehaviorPathPlannerParameters & param, const double front_decel, const double rear_decel,
  CollisionCheckDebug & debug);

bool isLateralDistanceEnough(
  const double & relative_lateral_distance, const double & lateral_distance_threshold);

bool isSafeInLaneletCollisionCheck(
  const Pose & ego_current_pose, const Twist & ego_current_twist,
  const PredictedPath & ego_predicted_path, const VehicleInfo & ego_info,
  const double check_start_time, const double check_end_time, const double check_time_resolution,
  const PredictedObject & target_object, const PredictedPath & target_object_path,
  const BehaviorPathPlannerParameters & common_parameters, const double front_decel,
  const double rear_decel, CollisionCheckDebug & debug);

bool isSafeInFreeSpaceCollisionCheck(
  const Pose & ego_current_pose, const Twist & ego_current_twist,
  const PredictedPath & ego_predicted_path, const VehicleInfo & ego_info,
  const double check_start_time, const double check_end_time, const double check_time_resolution,
  const PredictedObject & target_object, const BehaviorPathPlannerParameters & common_parameters,
  const double front_decel, const double rear_decel, CollisionCheckDebug & debug);

bool checkPathRelativeAngle(const PathWithLaneId & path, const double angle_threshold);
}  // namespace behavior_path_planner::util

#endif  // BEHAVIOR_PATH_PLANNER__UTILITIES_HPP_
