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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__TYPE_ALIAS_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__TYPE_ALIAS_HPP_

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"
#include "tier4_debug_msgs/msg/string_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace obstacle_avoidance_planner
{
// std_msgs
using std_msgs::msg::Header;
// planning
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
// navigation
using nav_msgs::msg::Odometry;
// visualization
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
// debug
using tier4_debug_msgs::msg::Float64Stamped;
using tier4_debug_msgs::msg::StringStamped;
}  // namespace obstacle_avoidance_planner

#endif  // OBSTACLE_AVOIDANCE_PLANNER__TYPE_ALIAS_HPP_
