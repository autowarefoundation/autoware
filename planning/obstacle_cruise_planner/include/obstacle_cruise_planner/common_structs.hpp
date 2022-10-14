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

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <string>
#include <vector>

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_perception_msgs::msg::Shape;

namespace
{
std::string toHexString(const unique_identifier_msgs::msg::UUID & id)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +id.uuid[i];
  }
  return ss.str();
}
}  // namespace

struct TargetObstacle
{
  TargetObstacle(
    const rclcpp::Time & arg_time_stamp, const PredictedObject & object,
    const double aligned_velocity,
    const std::vector<geometry_msgs::msg::PointStamped> & arg_collision_points)
  {
    time_stamp = arg_time_stamp;
    orientation_reliable = true;
    pose = object.kinematics.initial_pose_with_covariance.pose;
    velocity_reliable = true;
    velocity = aligned_velocity;
    is_classified = true;
    classification = object.classification.at(0);
    uuid = toHexString(object.object_id);

    predicted_paths.clear();
    for (const auto & path : object.kinematics.predicted_paths) {
      predicted_paths.push_back(path);
    }

    collision_points = arg_collision_points;
    has_stopped = false;
  }

  rclcpp::Time time_stamp;
  bool orientation_reliable;
  geometry_msgs::msg::Pose pose;
  bool velocity_reliable;
  float velocity;
  bool is_classified;
  ObjectClassification classification;
  std::string uuid;
  std::vector<PredictedPath> predicted_paths;
  std::vector<geometry_msgs::msg::PointStamped> collision_points;
  bool has_stopped;
};

struct ObstacleCruisePlannerData
{
  rclcpp::Time current_time;
  autoware_auto_planning_msgs::msg::Trajectory traj;
  geometry_msgs::msg::Pose current_pose;
  double current_vel;
  double current_acc;
  std::vector<TargetObstacle> target_obstacles;
  bool is_driving_forward;
};

struct LongitudinalInfo
{
  double max_accel;
  double min_accel;
  double max_jerk;
  double min_jerk;
  double limit_max_accel;
  double limit_min_accel;
  double limit_max_jerk;
  double limit_min_jerk;
  double idling_time;
  double min_ego_accel_for_rss;
  double min_object_accel_for_rss;
  double safe_distance_margin;
  double terminal_safe_distance_margin;
};

struct DebugData
{
  std::vector<PredictedObject> intentionally_ignored_obstacles;
  std::vector<TargetObstacle> obstacles_to_stop;
  std::vector<TargetObstacle> obstacles_to_cruise;
  visualization_msgs::msg::MarkerArray stop_wall_marker;
  visualization_msgs::msg::MarkerArray cruise_wall_marker;
  std::vector<tier4_autoware_utils::Polygon2d> detection_polygons;
  std::vector<geometry_msgs::msg::Point> collision_points;
};

struct EgoNearestParam
{
  EgoNearestParam(const double arg_dist_threshold, const double arg_yaw_threshold)
  : dist_threshold(arg_dist_threshold), yaw_threshold(arg_yaw_threshold)
  {
  }
  double dist_threshold;
  double yaw_threshold;
};

#endif  // OBSTACLE_CRUISE_PLANNER__COMMON_STRUCTS_HPP_
