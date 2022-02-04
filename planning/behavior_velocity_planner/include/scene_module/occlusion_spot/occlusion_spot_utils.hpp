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
#include <scene_module/occlusion_spot/geometry.hpp>
#include <scene_module/occlusion_spot/grid_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <utilization/util.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <visualization_msgs/msg/marker.hpp>

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

struct Sidewalk
{
  double focus_range;              // [m] distance to care about occlusion spot
  double slice_size;               // [m] size of each slice
  double min_occlusion_spot_size;  // [m] minumum size to care about the occlusion spot
};

struct VehicleInfo
{
  double vehicle_width;      // [m]  vehicle_width from parameter server
  double baselink_to_front;  // [m]  wheel_base + front_overhang
};

struct EgoVelocity
{
  double ebs_decel;     // [m/s^2] emergency braking system deceleration
  double pbs_decel;     // [m/s^2] predictive braking system deceleration
  double min_velocity;  // [m/s]   minimum allowed velocity not to stop
};

struct PlannerParam
{
  // parameters in yaml
  double safety_time_buffer;     // [s]
  double detection_area_length;  // [m]
  double stuck_vehicle_vel;      // [m/s]
  double lateral_distance_thr;   // [m] lateral distance threshold to consider
  double pedestrian_vel;         // [m/s]

  double dist_thr;       // [m]
  double angle_thr;      // [rad]
  bool show_debug_grid;  // [-]

  VehicleInfo vehicle_info;
  EgoVelocity private_road;
  EgoVelocity public_road;
  Sidewalk sidewalk;
  grid_utils::GridParam grid;
};

struct ObstacleInfo
{
  geometry_msgs::msg::Point position;
  double max_velocity;  // [m/s] Maximum velocity of the possible obstacle
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
  ObstacleInfo obstacle_info;  // For hidden obstacle
  autoware_auto_planning_msgs::msg::PathPoint
    collision_path_point;                              // For baselink at collision point
  geometry_msgs::msg::Pose intersection_pose;          // For egp path and hidden obstacle
  lanelet::ArcCoordinates arc_lane_dist_at_collision;  // For ego distance to obstacle in s-d
  PossibleCollisionInfo() = default;
  PossibleCollisionInfo(
    const ObstacleInfo & obstacle_info,
    const autoware_auto_planning_msgs::msg::PathPoint & collision_path_point,
    const geometry_msgs::msg::Pose & intersection_pose,
    const lanelet::ArcCoordinates & arc_lane_dist_to_occlusion)
  : obstacle_info(obstacle_info),
    collision_path_point(collision_path_point),
    intersection_pose(intersection_pose),
    arc_lane_dist_at_collision(arc_lane_dist_to_occlusion)
  {
  }
};

lanelet::ConstLanelet toPathLanelet(const PathWithLaneId & path);

// Note : consider offset_from_start_to_ego and safety margin for collision here
inline void handleCollisionOffset(
  std::vector<PossibleCollisionInfo> & possible_collisions, double offset, double margin)
{
  for (auto & pc : possible_collisions) {
    pc.arc_lane_dist_at_collision.length -= offset;
    pc.arc_lane_dist_at_collision.length -= margin;
  }
}

inline double offsetFromStartToEgo(
  const PathWithLaneId & path, const Pose & ego_pose, const int closest_idx)
{
  double offset_from_ego_to_closest = 0;
  for (int i = 0; i < closest_idx; i++) {
    const auto & curr_p = path.points.at(i).point.pose.position;
    const auto & next_p = path.points.at(i + 1).point.pose.position;
    offset_from_ego_to_closest += tier4_autoware_utils::calcDistance2d(curr_p, next_p);
  }
  const double offset_from_closest_to_target =
    -planning_utils::transformRelCoordinate2D(ego_pose, path.points[closest_idx].point.pose)
       .position.x;
  return offset_from_ego_to_closest + offset_from_closest_to_target;
}

inline void clipPathByLength(
  const PathWithLaneId & path, PathWithLaneId & clipped, const double max_length = 100.0)
{
  double length_sum = 0;
  for (int i = 0; i < static_cast<int>(path.points.size()) - 1; i++) {
    length_sum += tier4_autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));
    if (length_sum > max_length) return;
    clipped.points.emplace_back(path.points.at(i));
  }
}

inline bool isStuckVehicle(PredictedObject obj, const double min_vel)
{
  if (
    obj.classification.at(0).label == ObjectClassification::CAR ||
    obj.classification.at(0).label == ObjectClassification::TRUCK ||
    obj.classification.at(0).label == ObjectClassification::BUS) {
    if (std::abs(obj.kinematics.initial_twist_with_covariance.twist.linear.x) < min_vel) {
      return true;
    }
  }
  return false;
}
void filterCollisionByRoadType(
  std::vector<PossibleCollisionInfo> & possible_collisions, const DetectionAreaIdx road_type);
bool splineInterpolate(
  const PathWithLaneId & input, const double interval, PathWithLaneId * output,
  const rclcpp::Logger logger);
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
//!< @brief convert a set of occlusion spots found on sidewalk slice
void generateSidewalkPossibleCollisionFromOcclusionSpot(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const std::vector<grid_map::Position> & occlusion_spot_positions,
  const double offset_from_start_to_ego, const lanelet::ConstLanelet & path_lanelet,
  const PlannerParam & param);
//!< @brief generate possible collisions coming from occlusion spots on the side of the path
void generateSidewalkPossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const PathWithLaneId & path, const double offset_from_start_to_ego, const PlannerParam & param,
  std::vector<BasicPolygon2d> & debug);

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__OCCLUSION_SPOT_UTILS_HPP_
