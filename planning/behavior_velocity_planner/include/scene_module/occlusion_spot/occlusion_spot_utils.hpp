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

#ifndef SCENE_MODULE__OCCLUSION_SPOT__OCCLUSION_SPOT_UTILS_HPP_
#define SCENE_MODULE__OCCLUSION_SPOT__OCCLUSION_SPOT_UTILS_HPP_

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <scene_module/occlusion_spot/grid_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <utilization/util.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <chrono>
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
using DetectionAreaIdx = boost::optional<std::pair<double, double>>;

namespace occlusion_spot_utils
{
enum ROAD_TYPE { PRIVATE, PUBLIC, HIGHWAY, UNKNOWN };
enum METHOD { OCCUPANCY_GRID, PREDICTED_OBJECT };

struct DetectionArea
{
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
  METHOD method;
  bool debug;  // [-]
  // parameters in yaml
  double detection_area_length;      // [m]
  double detection_area_max_length;  // [m]
  double stuck_vehicle_vel;          // [m/s]
  double lateral_distance_thr;       // [m] lateral distance threshold to consider
  double pedestrian_vel;             // [m/s]

  double dist_thr;   // [m]
  double angle_thr;  // [rad]

  // vehicle info
  double half_vehicle_width;  // [m]  half vehicle_width from vehicle info
  double baselink_to_front;   // [m]  wheel_base + front_overhang

  Velocity v;
  DetectionArea detection_area;
  grid_utils::GridParam grid;
};

struct SafeMotion
{
  double stop_dist;
  double safe_velocity;
};

// @brief represent the range of a each polygon
struct SliceRange
{
  double min_length{};
  double max_length{};
  double min_distance{};
  double max_distance{};
};

// @brief representation of a polygon along a path
struct Slice
{
  SliceRange range{};
  lanelet::BasicPolygon2d polygon{};
};

struct ObstacleInfo
{
  SafeMotion safe_motion;  // safe motion of velocity and stop point
  geometry_msgs::msg::Point position;
  double max_velocity;  // [m/s] Maximum velocity of the possible obstacle
  double ttc;           // [s] time to collision with ego
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

lanelet::ConstLanelet toPathLanelet(const PathWithLaneId & path);
// Note : consider offset_from_start_to_ego and safety margin for collision here
void handleCollisionOffset(std::vector<PossibleCollisionInfo> & possible_collisions, double offset);
void clipPathByLength(
  const PathWithLaneId & path, PathWithLaneId & clipped, const double max_length = 100.0);
bool isStuckVehicle(PredictedObject obj, const double min_vel);
double offsetFromStartToEgo(
  const PathWithLaneId & path, const Pose & ego_pose, const int closest_idx);
std::vector<PredictedObject> filterDynamicObjectByDetectionArea(
  std::vector<PredictedObject> & objs, const std::vector<Slice> polys);
std::vector<PredictedObject> getParkedVehicles(
  const PredictedObjects & dyn_objects, const PlannerParam & param,
  std::vector<Point> & debug_point);
std::vector<PossibleCollisionInfo> generatePossibleCollisionBehindParkedVehicle(
  const PathWithLaneId & path, const PlannerParam & param, const double offset_from_start_to_ego,
  const std::vector<PredictedObject> & dyn_objects);
ROAD_TYPE getCurrentRoadType(
  const lanelet::ConstLanelet & current_lanelet, const LaneletMapPtr & lanelet_map_ptr);
//!< @brief calculate intersection and collision point from occlusion spot
void calculateCollisionPathPointFromOcclusionSpot(
  PossibleCollisionInfo & pc, const lanelet::BasicPoint2d & obstacle_point,
  const double offset_from_ego_to_target, const lanelet::ConstLanelet & path_lanelet,
  const PlannerParam & param);
//!< @brief create hidden collision behind parked car
void createPossibleCollisionBehindParkedVehicle(
  std::vector<PossibleCollisionInfo> & possible_collisions, const PathWithLaneId & path,
  const PlannerParam & param, const double offset_from_ego_to_target,
  const PredictedObjects::ConstSharedPtr & dyn_obj_arr);
//!< @brief set velocity and orientation to collision point based on previous Path with laneId
void calcSlowDownPointsForPossibleCollision(
  const int closest_idx, const PathWithLaneId & path, const double offset,
  std::vector<PossibleCollisionInfo> & possible_collisions);
//!< @brief extract lanelet that includes target_road_type only
DetectionAreaIdx extractTargetRoadArcLength(
  const LaneletMapPtr lanelet_map_ptr, const double max_range, const PathWithLaneId & path,
  const ROAD_TYPE & target_road_type);
//!< @brief convert a set of occlusion spots found on detection_area slice
boost::optional<PossibleCollisionInfo> generateOneNotableCollisionFromOcclusionSpot(
  const grid_map::GridMap & grid, const std::vector<grid_map::Position> & occlusion_spot_positions,
  const double offset_from_start_to_ego, const BasicPoint2d basic_point,
  const lanelet::ConstLanelet & path_lanelet, const PlannerParam & param);
//!< @brief generate possible collisions coming from occlusion spots on the side of the path
void createPossibleCollisionsInDetectionArea(
  const std::vector<Slice> & detection_area_polygons,
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const PathWithLaneId & path, const double offset_from_start_to_ego, const PlannerParam & param,
  std::vector<Point> & debug_points);

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__OCCLUSION_SPOT_UTILS_HPP_
