// Copyright 2020 Tier IV, Inc.
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
#ifndef OBSTACLE_AVOIDANCE_PLANNER__UTILS__DEBUG_UTILS_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__UTILS__DEBUG_UTILS_HPP_

#include "obstacle_avoidance_planner/common_structs.hpp"
#include "opencv2/core.hpp"
#include "rclcpp/clock.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/ros/marker_helper.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <memory>
#include <string>
#include <vector>

using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;

namespace debug_utils
{
visualization_msgs::msg::MarkerArray getDebugVisualizationMarker(
  DebugData & debug_data,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & optimized_points,
  const VehicleParam & vehicle_param, const bool is_showing_debug_detail);

visualization_msgs::msg::MarkerArray getDebugVisualizationWallMarker(
  DebugData & debug_data, const VehicleParam & vehicle_param);

nav_msgs::msg::OccupancyGrid getDebugCostmap(
  const cv::Mat & clearance_map, const nav_msgs::msg::OccupancyGrid & occupancy_grid);
}  // namespace debug_utils

#endif  // OBSTACLE_AVOIDANCE_PLANNER__UTILS__DEBUG_UTILS_HPP_
