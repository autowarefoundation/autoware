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
#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_path.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
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

#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace tier4_autoware_utils
{
template <>
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose;
}
}  // namespace tier4_autoware_utils

namespace tf2
{
inline void fromMsg(const geometry_msgs::msg::PoseStamped & msg, tf2::Stamped<tf2::Transform> & out)
{
  out.stamp_ = tf2_ros::fromMsg(msg.header.stamp);
  out.frame_id_ = msg.header.frame_id;
  tf2::Transform tmp;
  fromMsg(msg.pose, tmp);
  out.setData(tmp);
}

// Remove after this commit is released
// https://github.com/ros2/geometry2/commit/e9da371d81e388a589540357c050e262442f1b4a
inline geometry_msgs::msg::Point & toMsg(const tf2::Vector3 & in, geometry_msgs::msg::Point & out)
{
  out.x = in.getX();
  out.y = in.getY();
  out.z = in.getZ();
  return out;
}

// Remove after this commit is released
// https://github.com/ros2/geometry2/commit/e9da371d81e388a589540357c050e262442f1b4a
inline void fromMsg(const geometry_msgs::msg::Point & in, tf2::Vector3 & out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

template <>
inline void doTransform(
  const geometry_msgs::msg::Point & t_in, geometry_msgs::msg::Point & t_out,
  const geometry_msgs::msg::TransformStamped & transform)
{
  tf2::Transform t;
  fromMsg(transform.transform, t);
  tf2::Vector3 v_in;
  fromMsg(t_in, v_in);
  tf2::Vector3 v_out = t * v_in;
  toMsg(v_out, t_out);
}
}  // namespace tf2

namespace behavior_path_planner
{
namespace util
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
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

struct FrenetCoordinate3d
{
  double length{0.0};    // longitudinal
  double distance{0.0};  // lateral
};

// data conversions

Path convertToPathFromPathWithLaneId(const PathWithLaneId & path_with_lane_id);

std::vector<Point> convertToPointArray(const PathWithLaneId & path);

std::vector<Point> convertToGeometryPointArray(const PathWithLaneId & path);

PoseArray convertToGeometryPoseArray(const PathWithLaneId & path);

PredictedPath convertToPredictedPath(
  const PathWithLaneId & path, const Twist & vehicle_twist, const Pose & vehicle_pose,
  const double duration, const double resolution, const double acceleration);

bool convertToFrenetCoordinate3d(
  const PathWithLaneId & path, const Point & search_point_geom,
  FrenetCoordinate3d * frenet_coordinate);

bool convertToFrenetCoordinate3d(
  const std::vector<Point> & linestring, const Point search_point_geom,
  FrenetCoordinate3d * frenet_coordinate);

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

Point lerpByLength(const std::vector<Point> & array, const double length);

bool lerpByTimeStamp(const PredictedPath & path, const double t, Pose * lerped_pt);

bool lerpByDistance(
  const behavior_path_planner::PullOutPath & path, const double & s, Pose * lerped_pt,
  const lanelet::ConstLanelets & road_lanes);

bool calcObjectPolygon(const PredictedObject & object, Polygon2d * object_polygon);

PredictedPath resamplePredictedPath(
  const PredictedPath & input_path, const double resolution, const double duration);

double getDistanceBetweenPredictedPaths(
  const PredictedPath & path1, const PredictedPath & path2, const double start_time,
  const double end_time, const double resolution);

double getDistanceBetweenPredictedPathAndObject(
  const PredictedObject & object, const PredictedPath & path, const double start_time,
  const double end_time, const double resolution);

double getDistanceBetweenPredictedPathAndObjectPolygon(
  const PredictedObject & object, const PullOutPath & ego_path,
  const tier4_autoware_utils::LinearRing2d & vehicle_footprint, double distance_resolution,
  const lanelet::ConstLanelets & road_lanes);

/**
 * @brief Get index of the obstacles inside the lanelets with start and end length
 * @return Indices corresponding to the obstacle inside the lanelets
 */
std::vector<size_t> filterObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & lanelets,
  const double start_arc_length, const double end_arc_length);

/**
 * @brief Get index of the obstacles inside the lanelets
 * @return Indices corresponding to the obstacle inside the lanelets
 */
std::vector<size_t> filterObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & target_lanelets);

std::vector<size_t> filterObjectsByPath(
  const PredictedObjects & objects, const std::vector<size_t> & object_indices,
  const PathWithLaneId & ego_path, const double vehicle_width);

PredictedObjects filterObjectsByVelocity(const PredictedObjects & objects, double lim_v);

PredictedObjects filterObjectsByVelocity(
  const PredictedObjects & objects, double min_v, double max_v);

// drivable area generation

void occupancyGridToImage(const OccupancyGrid & occupancy_grid, cv::Mat * cv_image);

void imageToOccupancyGrid(const cv::Mat & cv_image, OccupancyGrid * occupancy_grid);

cv::Point toCVPoint(
  const Point & geom_point, const double width_m, const double height_m, const double resolution);

OccupancyGrid generateDrivableArea(
  const lanelet::ConstLanelets & lanes, const PoseStamped & current_pose, const double width,
  const double height, const double resolution, const double vehicle_length,
  const RouteHandler & route_handler);

// goal management

/**
 * @brief Modify the path points near the goal to smoothly connect the input path and the goal point
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

double getDistanceToShoulderBoundary(
  const lanelet::ConstLanelets & shoulder_lanelets, const Pose & pose);

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
  const BehaviorPathPlannerParameters & parameter);

PathWithLaneId setDecelerationVelocity(
  const RouteHandler & route_handler, const PathWithLaneId & input,
  const lanelet::ConstLanelets & lanelet_sequence, const double lane_change_prepare_duration,
  const double lane_change_buffer);

PathWithLaneId setDecelerationVelocity(
  const RouteHandler & route_handler, const PathWithLaneId & input,
  const lanelet::ConstLanelets & lanelet_sequence, const double distance_after_pullover,
  const double pullover_distance_min, const double distance_before_pull_over,
  const double deceleration_interval, Pose goal_pose);

// object label
std::uint8_t getHighestProbLabel(const std::vector<ObjectClassification> & classification);

}  // namespace util
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILITIES_HPP_
