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

#ifndef BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__DEBUG_HPP_
#define BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__DEBUG_HPP_

#include <rclcpp/time.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace debug
{
visualization_msgs::msg::MarkerArray createPolygonMarkerArray(
  const geometry_msgs::msg::Polygon & polygon, const std::string & ns, const int64_t module_id,
  const rclcpp::Time & now, const double x, const double y, const double z, const double r,
  const double g, const double b);
visualization_msgs::msg::MarkerArray createPathMarkerArray(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const rclcpp::Time & now, const double x, const double y, const double z,
  const double r, const double g, const double b);
visualization_msgs::msg::MarkerArray createObjectsMarkerArray(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects, const std::string & ns,
  const int64_t module_id, const rclcpp::Time & now, const double r, const double g,
  const double b);
visualization_msgs::msg::MarkerArray createPointsMarkerArray(
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & ns,
  const int64_t module_id, const rclcpp::Time & now, const double x, const double y, const double z,
  const double r, const double g, const double b);
}  // namespace debug
}  // namespace behavior_velocity_planner
#endif  // BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__DEBUG_HPP_
