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

#ifndef OCCLUSION_SPOT_UTILS_HPP_
#define OCCLUSION_SPOT_UTILS_HPP_

#include "grid_utils.hpp"

#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using lanelet::ArcCoordinates;
using lanelet::BasicLineString2d;
using lanelet::BasicPoint2d;
using lanelet::BasicPolygon2d;
using lanelet::ConstLineString2d;
using lanelet::LaneletMapPtr;
using lanelet::geometry::fromArcCoordinates;
using lanelet::geometry::toArcCoordinates;
using DetectionAreaIdx = std::optional<std::pair<double, double>>;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;

namespace occlusion_spot_utils
{
enum ROAD_TYPE { PRIVATE, PUBLIC, HIGHWAY, UNKNOWN };
enum DETECTION_METHOD { OCCUPANCY_GRID, PREDICTED_OBJECT };
enum PASS_JUDGE { SMOOTH_VELOCITY, CURRENT_VELOCITY };

struct DetectionArea
{
  double min_longitudinal_offset;  // [m] detection area safety buffer from front bumper
  double max_lateral_distance;     // [m] distance to care about occlusion spot
  double slice_length;             // [m] size of each slice
  double min_occlusion_spot_size;  // [m] minumum size to care about the occlusion spot
};
struct Velocity
{
  double safety_ratio;          // [-] safety margin for planning error
  double max_stop_jerk;         // [m/s^3] emergency braking system jerk
  double max_stop_accel;        // [m/s^2] emergency braking system deceleration
  double max_slow_down_jerk;    // [m/s^3] maximum allowed slowdown jerk
  double max_slow_down_accel;   // [m/s^2] maximum allowed deceleration
  double non_effective_jerk;    // [m/s^3] too weak jerk for velocity planning.
  double non_effective_accel;   // [m/s^2] too weak deceleration for velocity planning.
  double min_allowed_velocity;  // [m/s]   minimum allowed velocity not to stop
  double a_ego;                 // [m/s^2] current ego acceleration
  double v_ego;                 // [m/s]   current ego velocity
  double delay_time;            // [s] safety time buffer for delay response
  double safe_margin;           // [m] maximum safety distance for any error
};

struct LatLon
{
  double lateral_distance;       // [m] lateral distance
  double longitudinal_distance;  // [m] longitudinal distance
};

struct PlannerParam
{
  DETECTION_METHOD detection_method;
  PASS_JUDGE pass_judge;
  bool is_show_occlusion;           // [-]
  bool is_show_cv_window;           // [-]
  bool is_show_processing_time;     // [-]
  bool use_object_info;             // [-]
  bool use_moving_object_ray_cast;  // [-]
  bool use_partition_lanelet;       // [-]
  // parameters in yaml
  double detection_area_offset;      // [m]
  double detection_area_length;      // [m]
  double detection_area_max_length;  // [m]
  double stuck_vehicle_vel;          // [m/s]
  double lateral_distance_thr;       // [m] lateral distance threshold to consider
  double pedestrian_vel;             // [m/s]
  double pedestrian_radius;          // [m]

  double dist_thr;   // [m]
  double angle_thr;  // [rad]

  // vehicle info
  double baselink_to_front;  // [m]  wheel_base + front_overhang
  double wheel_tread;        // [m]  wheel_tread from vehicle info
  double right_overhang;     // [m]  right_overhang from vehicle info
  double left_overhang;      // [m]  left_overhang from vehicle info

  Velocity v;
  DetectionArea detection_area;
  grid_utils::GridParam grid;
};

struct SafeMotion
{
  double stop_dist;
  double safe_velocity;
};

struct ObstacleInfo
{
  SafeMotion safe_motion;  // safe motion of velocity and stop point
  geometry_msgs::msg::Point position;
  double max_velocity;  // [m/s] Maximum velocity of the possible obstacle
  double ttv;           // [s] time to vehicle for pedestrian
};

/**
 * @brief representation of a possible collision between ego and some obstacle
 *                                      ^
 *                                      |
 * Ego ---------collision----------intersection-------> path
 *                                      |
 *             ------------------       |
 *            |     Vehicle      |   obstacle
 *             ------------------
 */
struct PossibleCollisionInfo
{
  ObstacleInfo obstacle_info;                          // For hidden obstacle
  PathPoint collision_with_margin;                     // For baselink at collision point
  Pose collision_pose;                                 // only use this for debugging
  Pose intersection_pose;                              // For egp path and hidden obstacle
  lanelet::ArcCoordinates arc_lane_dist_at_collision;  // For ego distance to obstacle in s-d
  PossibleCollisionInfo() = default;
  PossibleCollisionInfo(
    const ObstacleInfo & obstacle_info, const PathPoint & collision_with_margin,
    const Pose & intersection_pose, const lanelet::ArcCoordinates & arc_lane_dist_to_occlusion)
  : obstacle_info(obstacle_info),
    collision_with_margin(collision_with_margin),
    intersection_pose(intersection_pose),
    arc_lane_dist_at_collision(arc_lane_dist_to_occlusion)
  {
  }
};

struct DebugData
{
  double z;
  double baselink_to_front;
  std::string road_type = "";
  std::string detection_type = "";
  Polygons2d detection_area_polygons;
  std::vector<lanelet::BasicPolygon2d> close_partition;
  std::vector<geometry_msgs::msg::Point> parked_vehicle_point;
  std::vector<PossibleCollisionInfo> possible_collisions;
  std::vector<geometry_msgs::msg::Point> occlusion_points;
  std::vector<geometry_msgs::msg::Pose> debug_poses;
  void resetData()
  {
    debug_poses.clear();
    close_partition.clear();
    detection_area_polygons.clear();
    parked_vehicle_point.clear();
    possible_collisions.clear();
    occlusion_points.clear();
  }
};
// apply current velocity to path
PathWithLaneId applyVelocityToPath(const PathWithLaneId & path, const double v0);
//!< @brief wrapper for detection area polygon generation
bool buildDetectionAreaPolygon(
  Polygons2d & slices, const PathWithLaneId & path, const geometry_msgs::msg::Pose & target_pose,
  const size_t target_seg_idx, const PlannerParam & param);
lanelet::ConstLanelet toPathLanelet(const PathWithLaneId & path);
// Note : consider offset_from_start_to_ego and safety margin for collision here
void handleCollisionOffset(std::vector<PossibleCollisionInfo> & possible_collisions, double offset);
void clipPathByLength(
  const PathWithLaneId & path, PathWithLaneId & clipped, const double max_length = 100.0);
//!< @brief extract target vehicles
bool isStuckVehicle(const PredictedObject & obj, const double min_vel);
bool isMovingVehicle(const PredictedObject & obj, const double min_vel);
std::vector<PredictedObject> extractVehicles(
  const PredictedObjects::ConstSharedPtr objects_ptr, const Point ego_position,
  const double distance);
std::vector<PredictedObject> filterVehiclesByDetectionArea(
  const std::vector<PredictedObject> & objs, const Polygons2d & polys);
bool isVehicle(const ObjectClassification & obj_class);
void categorizeVehicles(
  const std::vector<PredictedObject> & vehicles, Polygons2d & stuck_vehicle_foot_prints,
  Polygons2d & moving_vehicle_foot_prints, const double stuck_vehicle_vel);
bool generatePossibleCollisionsFromObjects(
  std::vector<PossibleCollisionInfo> & possible_collisions, const PathWithLaneId & path,
  const PlannerParam & param, const double offset_from_start_to_ego,
  const std::vector<PredictedObject> & dyn_objects);
//!< @brief calculate intersection and collision point from occlusion spot
void calculateCollisionPathPointFromOcclusionSpot(
  PossibleCollisionInfo & pc, const lanelet::BasicPoint2d & obstacle_point,
  const double offset_from_ego_to_target, const lanelet::ConstLanelet & path_lanelet,
  const PlannerParam & param);
//!< @brief set velocity and orientation to collision point based on previous Path with laneId
void calcSlowDownPointsForPossibleCollision(
  const int closest_idx, const PathWithLaneId & path, const double offset,
  std::vector<PossibleCollisionInfo> & possible_collisions);
//!< @brief convert a set of occlusion spots found on detection_area slice
std::optional<PossibleCollisionInfo> generateOneNotableCollisionFromOcclusionSpot(
  const grid_map::GridMap & grid, const std::vector<grid_map::Position> & occlusion_spot_positions,
  const double offset_from_start_to_ego, const Point2d base_point,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param, DebugData & debug_data);
//!< @brief generate possible collisions coming from occlusion spots on the side of the path
bool generatePossibleCollisionsFromGridMap(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const PathWithLaneId & path, const double offset_from_start_to_ego, const PlannerParam & param,
  DebugData & debug_data);

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner

#endif  // OCCLUSION_SPOT_UTILS_HPP_
