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

#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include "object_stop_decision.hpp"
#include "types.hpp"

#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop::debug
{
std::vector<visualization_msgs::msg::Marker> make_delete_markers(
  const size_t from, const size_t to, const std::string & ns);
void add_markers(
  visualization_msgs::msg::MarkerArray & array, size_t & prev_nb,
  const std::vector<visualization_msgs::msg::Marker> & markers, const std::string & ns);
std::vector<visualization_msgs::msg::Marker> make_collision_markers(
  const ObjectStopDecisionMap & object_map, const std::string & ns, const double z,
  const rclcpp::Time & now);
std::vector<visualization_msgs::msg::Marker> make_polygon_markers(
  const autoware::universe_utils::MultiPolygon2d & footprints, const std::string & ns,
  const double z);
}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop::debug

#endif  // DEBUG_HPP_
