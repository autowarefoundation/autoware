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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__PATH_SAFETY_CHECKER_PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__PATH_SAFETY_CHECKER_PARAMETERS_HPP_

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner::utils::path_safety_checker
{

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using tier4_autoware_utils::Polygon2d;

struct PoseWithVelocity
{
  Pose pose;
  double velocity{0.0};

  PoseWithVelocity(const Pose & pose, const double velocity) : pose(pose), velocity(velocity) {}
};

struct PoseWithVelocityStamped : public PoseWithVelocity
{
  double time{0.0};

  PoseWithVelocityStamped(const double time, const Pose & pose, const double velocity)
  : PoseWithVelocity(pose, velocity), time(time)
  {
  }
};

struct PoseWithVelocityAndPolygonStamped : public PoseWithVelocityStamped
{
  Polygon2d poly;

  PoseWithVelocityAndPolygonStamped(
    const double time, const Pose & pose, const double velocity, const Polygon2d & poly)
  : PoseWithVelocityStamped(time, pose, velocity), poly(poly)
  {
  }
};

struct PredictedPathWithPolygon
{
  float confidence{0.0};
  std::vector<PoseWithVelocityAndPolygonStamped> path;
};

struct ExtendedPredictedObject
{
  unique_identifier_msgs::msg::UUID uuid;
  geometry_msgs::msg::PoseWithCovariance initial_pose;
  geometry_msgs::msg::TwistWithCovariance initial_twist;
  geometry_msgs::msg::AccelWithCovariance initial_acceleration;
  autoware_auto_perception_msgs::msg::Shape shape;
  std::vector<PredictedPathWithPolygon> predicted_paths;
};

/**
 * @brief Specifies which object class should be checked.
 */
struct ObjectTypesToCheck
{
  bool check_car{true};         ///< Check for cars.
  bool check_truck{true};       ///< Check for trucks.
  bool check_bus{true};         ///< Check for buses.
  bool check_trailer{true};     ///< Check for trailers.
  bool check_unknown{true};     ///< Check for unknown object types.
  bool check_bicycle{true};     ///< Check for bicycles.
  bool check_motorcycle{true};  ///< Check for motorcycles.
  bool check_pedestrian{true};  ///< Check for pedestrians.
};

/**
 * @brief Configuration for which lanes should be checked for objects.
 */
struct ObjectLaneConfiguration
{
  bool check_current_lane{};   ///< Check the current lane.
  bool check_right_lane{};     ///< Check the right lane.
  bool check_left_lane{};      ///< Check the left lane.
  bool check_shoulder_lane{};  ///< Check the shoulder lane.
  bool check_other_lane{};     ///< Check other lanes.
};

/**
 * @brief Contains objects on lanes type.
 */
struct TargetObjectsOnLane
{
  std::vector<ExtendedPredictedObject> on_current_lane{};   ///< Objects on the current lane.
  std::vector<ExtendedPredictedObject> on_right_lane{};     ///< Objects on the right lane.
  std::vector<ExtendedPredictedObject> on_left_lane{};      ///< Objects on the left lane.
  std::vector<ExtendedPredictedObject> on_shoulder_lane{};  ///< Objects on the shoulder lane.
  std::vector<ExtendedPredictedObject> on_other_lane{};     ///< Objects on other lanes.
};

/**
 * @brief Parameters related to the RSS (Responsibility-Sensitive Safety) model.
 */
struct RSSparams
{
  double rear_vehicle_reaction_time{0.0};       ///< Reaction time of the rear vehicle.
  double rear_vehicle_safety_time_margin{0.0};  ///< Safety time margin for the rear vehicle.
  double lateral_distance_max_threshold{0.0};   ///< Maximum threshold for lateral distance.
  double longitudinal_distance_min_threshold{
    0.0};                                        ///< Minimum threshold for longitudinal distance.
  double longitudinal_velocity_delta_time{0.0};  ///< Delta time for longitudinal velocity.
  double front_vehicle_deceleration;             ///< brake parameter
  double rear_vehicle_deceleration;              ///< brake parameter
};

/**
 * @brief Parameters for generating the ego vehicle's predicted path.
 */
struct EgoPredictedPathParams
{
  double acceleration;           ///< Acceleration value.
  double time_horizon;           ///< Time horizon for prediction.
  double time_resolution;        ///< Time resolution for prediction.
  double min_slow_speed;         ///< Minimum slow speed.
  double delay_until_departure;  ///< Delay before departure.
  double target_velocity;        ///< Target velocity.
};

/**
 * @brief Parameters for filtering objects.
 */
struct ObjectsFilteringParams
{
  double safety_check_time_horizon;                   ///< Time horizon for object's prediction.
  double safety_check_time_resolution;                ///< Time resolution for object's prediction.
  double object_check_forward_distance;               ///< Forward distance for object checks.
  double object_check_backward_distance;              ///< Backward distance for object checks.
  double ignore_object_velocity_threshold;            ///< Velocity threshold for ignoring objects.
  ObjectTypesToCheck object_types_to_check;           ///< Specifies which object types to check.
  ObjectLaneConfiguration object_lane_configuration;  ///< Configuration for which lanes to check.
  bool include_opposite_lane;                         ///< Include the opposite lane in checks.
  bool invert_opposite_lane;                          ///< Invert the opposite lane in checks.
  bool check_all_predicted_path;                      ///< Check all predicted paths.
  bool use_all_predicted_path;                        ///< Use all predicted paths.
  bool use_predicted_path_outside_lanelet;            ///< Use predicted paths outside of lanelets.
};

/**
 * @brief Parameters for safety checks.
 */
struct SafetyCheckParams
{
  bool enable_safety_check;          ///< Enable safety checks.
  double backward_lane_length;       ///< Length of the backward lane for path generation.
  double forward_path_length;        ///< Length of the forward path lane for path generation.
  RSSparams rss_params;              ///< Parameters related to the RSS model.
  bool publish_debug_marker{false};  ///< Option to publish debug markers.
};

struct CollisionCheckDebug
{
  std::string unsafe_reason;                ///< Reason indicating unsafe situation.
  Pose current_pose{};                      ///< Ego vehicle's current pose.
  Twist current_twist{};                    ///< Ego vehicle's current velocity and rotation.
  Twist object_twist{};                     ///< Detected object's velocity and rotation.
  Pose expected_ego_pose{};                 ///< Predicted future pose of ego vehicle.
  Pose expected_obj_pose{};                 ///< Predicted future pose of object.
  double rss_longitudinal{0.0};             ///< Longitudinal RSS measure.
  double inter_vehicle_distance{0.0};       ///< Distance between ego vehicle and object.
  double extended_polygon_lon_offset{0.0};  ///< Longitudinal offset for extended polygon.
  double extended_polygon_lat_offset{0.0};  ///< Lateral offset for extended polygon.
  bool is_front{false};                     ///< True if object is in front of ego vehicle.
  bool is_safe{false};                      ///< True if situation is deemed safe.
  std::vector<Pose> lerped_path;            ///< Interpolated ego vehicle path.
  Polygon2d extended_ego_polygon{};         ///< Ego vehicle's extended collision polygon.
  Polygon2d extended_obj_polygon{};         ///< Detected object's extended collision polygon.
};
using CollisionCheckDebugPair = std::pair<std::string, CollisionCheckDebug>;
using CollisionCheckDebugMap =
  std::unordered_map<CollisionCheckDebugPair::first_type, CollisionCheckDebugPair::second_type>;

}  // namespace behavior_path_planner::utils::path_safety_checker

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__PATH_SAFETY_CHECKER__PATH_SAFETY_CHECKER_PARAMETERS_HPP_
