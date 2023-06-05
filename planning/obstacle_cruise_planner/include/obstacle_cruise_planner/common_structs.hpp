// Copyright 2022 TIER IV, Inc.
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

#ifndef OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
#define OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_

#include "motion_utils/motion_utils.hpp"
#include "obstacle_cruise_planner/type_alias.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>
#include <vector>

struct PlannerData
{
  rclcpp::Time current_time;
  std::vector<TrajectoryPoint> traj_points;
  geometry_msgs::msg::Pose ego_pose;
  double ego_vel;
  double ego_acc;
  bool is_driving_forward;
};

struct PoseWithStamp
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;
};

struct PointWithStamp
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Point point;
};

struct Obstacle
{
  Obstacle(
    const rclcpp::Time & arg_stamp, const PredictedObject & object,
    const geometry_msgs::msg::Pose & arg_pose)
  : stamp(arg_stamp),
    pose(arg_pose),
    orientation_reliable(true),
    twist(object.kinematics.initial_twist_with_covariance.twist),
    twist_reliable(true),
    classification(object.classification.at(0)),
    uuid(tier4_autoware_utils::toHexString(object.object_id)),
    shape(object.shape)
  {
    predicted_paths.clear();
    for (const auto & path : object.kinematics.predicted_paths) {
      predicted_paths.push_back(path);
    }
  }

  Polygon2d toPolygon() const { return tier4_autoware_utils::toPolygon2d(pose, shape); }

  rclcpp::Time stamp;  // This is not the current stamp, but when the object was observed.
  geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
  bool orientation_reliable;
  Twist twist;
  bool twist_reliable;
  ObjectClassification classification;
  std::string uuid;
  Shape shape;
  std::vector<PredictedPath> predicted_paths;
};

struct TargetObstacleInterface
{
  TargetObstacleInterface(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const geometry_msgs::msg::Pose & arg_pose, const double arg_velocity,
    const double arg_lat_velocity)
  : uuid(arg_uuid),
    stamp(arg_stamp),
    pose(arg_pose),
    velocity(arg_velocity),
    lat_velocity(arg_lat_velocity)
  {
  }
  std::string uuid;
  rclcpp::Time stamp;
  geometry_msgs::msg::Pose pose;  // interpolated with the current stamp
  double velocity;                // longitudinal velocity against ego's trajectory
  double lat_velocity;            // lateral velocity against ego's trajectory
};

struct StopObstacle : public TargetObstacleInterface
{
  StopObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const geometry_msgs::msg::Pose & arg_pose, const double arg_lon_velocity,
    const double arg_lat_velocity, const geometry_msgs::msg::Point arg_collision_point)
  : TargetObstacleInterface(arg_uuid, arg_stamp, arg_pose, arg_lon_velocity, arg_lat_velocity),
    collision_point(arg_collision_point)
  {
  }
  geometry_msgs::msg::Point collision_point;
};

struct CruiseObstacle : public TargetObstacleInterface
{
  CruiseObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const geometry_msgs::msg::Pose & arg_pose, const double arg_lon_velocity,
    const double arg_lat_velocity, const std::vector<PointWithStamp> & arg_collision_points)
  : TargetObstacleInterface(arg_uuid, arg_stamp, arg_pose, arg_lon_velocity, arg_lat_velocity),
    collision_points(arg_collision_points)
  {
  }
  std::vector<PointWithStamp> collision_points;  // time-series collision points
};

struct SlowDownObstacle : public TargetObstacleInterface
{
  SlowDownObstacle(
    const std::string & arg_uuid, const rclcpp::Time & arg_stamp,
    const geometry_msgs::msg::Pose & arg_pose, const double arg_lon_velocity,
    const double arg_lat_velocity, const double arg_precise_lat_dist,
    const geometry_msgs::msg::Point & arg_front_collision_point,
    const geometry_msgs::msg::Point & arg_back_collision_point)
  : TargetObstacleInterface(arg_uuid, arg_stamp, arg_pose, arg_lon_velocity, arg_lat_velocity),
    precise_lat_dist(arg_precise_lat_dist),
    front_collision_point(arg_front_collision_point),
    back_collision_point(arg_back_collision_point)
  {
  }
  double precise_lat_dist;  // for efficient calculation
  geometry_msgs::msg::Point front_collision_point;
  geometry_msgs::msg::Point back_collision_point;
};

struct LongitudinalInfo
{
  explicit LongitudinalInfo(rclcpp::Node & node)
  {
    max_accel = node.declare_parameter<double>("normal.max_acc");
    min_accel = node.declare_parameter<double>("normal.min_acc");
    max_jerk = node.declare_parameter<double>("normal.max_jerk");
    min_jerk = node.declare_parameter<double>("normal.min_jerk");
    limit_max_accel = node.declare_parameter<double>("limit.max_acc");
    limit_min_accel = node.declare_parameter<double>("limit.min_acc");
    limit_max_jerk = node.declare_parameter<double>("limit.max_jerk");
    limit_min_jerk = node.declare_parameter<double>("limit.min_jerk");

    idling_time = node.declare_parameter<double>("common.idling_time");
    min_ego_accel_for_rss = node.declare_parameter<double>("common.min_ego_accel_for_rss");
    min_object_accel_for_rss = node.declare_parameter<double>("common.min_object_accel_for_rss");

    safe_distance_margin = node.declare_parameter<double>("common.safe_distance_margin");
    terminal_safe_distance_margin =
      node.declare_parameter<double>("common.terminal_safe_distance_margin");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    tier4_autoware_utils::updateParam<double>(parameters, "normal.max_accel", max_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "normal.min_accel", min_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "normal.max_jerk", max_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "normal.min_jerk", min_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.max_accel", limit_max_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.min_accel", limit_min_accel);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.max_jerk", limit_max_jerk);
    tier4_autoware_utils::updateParam<double>(parameters, "limit.min_jerk", limit_min_jerk);

    tier4_autoware_utils::updateParam<double>(parameters, "common.idling_time", idling_time);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.min_ego_accel_for_rss", min_ego_accel_for_rss);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.min_object_accel_for_rss", min_object_accel_for_rss);

    tier4_autoware_utils::updateParam<double>(
      parameters, "common.safe_distance_margin", safe_distance_margin);
    tier4_autoware_utils::updateParam<double>(
      parameters, "common.terminal_safe_distance_margin", terminal_safe_distance_margin);
  }

  // common parameter
  double max_accel;
  double min_accel;
  double max_jerk;
  double min_jerk;
  double limit_max_accel;
  double limit_min_accel;
  double limit_max_jerk;
  double limit_min_jerk;

  // rss parameter
  double idling_time;
  double min_ego_accel_for_rss;
  double min_object_accel_for_rss;

  // distance margin
  double safe_distance_margin;
  double terminal_safe_distance_margin;
};

struct DebugData
{
  DebugData() = default;
  std::vector<Obstacle> intentionally_ignored_obstacles;
  std::vector<StopObstacle> obstacles_to_stop;
  std::vector<CruiseObstacle> obstacles_to_cruise;
  std::vector<SlowDownObstacle> obstacles_to_slow_down;
  MarkerArray stop_wall_marker;
  MarkerArray cruise_wall_marker;
  MarkerArray slow_down_wall_marker;
  std::vector<tier4_autoware_utils::Polygon2d> detection_polygons;
};

struct EgoNearestParam
{
  EgoNearestParam() = default;
  explicit EgoNearestParam(rclcpp::Node & node)
  {
    dist_threshold = node.declare_parameter<double>("ego_nearest_dist_threshold");
    yaw_threshold = node.declare_parameter<double>("ego_nearest_yaw_threshold");
  }

  TrajectoryPoint calcInterpolatedPoint(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return motion_utils::calcInterpolatedPoint(
      motion_utils::convertToTrajectory(traj_points), pose, dist_threshold, yaw_threshold);
  }

  size_t findIndex(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      traj_points, pose, dist_threshold, yaw_threshold);
  }

  size_t findSegmentIndex(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & pose) const
  {
    return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      traj_points, pose, dist_threshold, yaw_threshold);
  }

  double dist_threshold;
  double yaw_threshold;
};

#endif  // OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
