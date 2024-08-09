// Copyright 2023-2024 TIER IV, Inc.
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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
using TrajectoryPoints = std::vector<autoware_planning_msgs::msg::TrajectoryPoint>;
using BoxIndexPair = std::pair<autoware::universe_utils::Box2d, size_t>;
using Rtree = boost::geometry::index::rtree<BoxIndexPair, boost::geometry::index::rstar<16, 4>>;

/// @brief parameters for the "out of lane" module
struct PlannerParam
{
  double extra_object_width{};
  double minimum_object_velocity{};
  double stop_distance_buffer{};
  double time_horizon{};
  double hysteresis{};
  double add_duration_buffer{};
  double remove_duration_buffer{};
  double ego_longitudinal_offset{};
  double ego_lateral_offset{};
  double minimum_object_distance_from_ego_trajectory{};
  bool ignore_unavoidable_collisions{};
};

struct EgoData
{
  TrajectoryPoints trajectory;
  size_t first_trajectory_idx{};
  double longitudinal_offset_to_first_trajectory_idx;  // [m]
  geometry_msgs::msg::Pose pose;
  autoware::universe_utils::MultiPolygon2d trajectory_footprints;
  Rtree rtree;
  std::optional<geometry_msgs::msg::Pose> earliest_stop_pose;
};

/// @brief debug data
struct DebugData
{
  autoware::universe_utils::MultiPolygon2d obstacle_footprints;
  size_t prev_dynamic_obstacles_nb{};
  autoware::universe_utils::MultiPolygon2d ego_footprints;
  size_t prev_ego_footprints_nb{};
  std::optional<geometry_msgs::msg::Pose> stop_pose;
  size_t prev_collisions_nb{};
  double z{};
  void reset_data()
  {
    obstacle_footprints.clear();
    ego_footprints.clear();
    stop_pose.reset();
  }
};

struct Collision
{
  geometry_msgs::msg::Point point;
  std::string object_uuid;
};
}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop

#endif  // TYPES_HPP_
