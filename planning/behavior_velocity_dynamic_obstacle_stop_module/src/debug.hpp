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

#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include "types.hpp"

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <string>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop::debug
{
std::vector<visualization_msgs::msg::Marker> make_delete_markers(
  const size_t from, const size_t to, const std::string & ns);
std::vector<visualization_msgs::msg::Marker> make_dynamic_obstacle_markers(
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & obstacles);
std::vector<visualization_msgs::msg::Marker> make_polygon_markers(
  const tier4_autoware_utils::MultiPolygon2d & footprints, const std::string & ns, const double z);
std::vector<visualization_msgs::msg::Marker> make_collision_markers(
  const tier4_autoware_utils::MultiPoint2d & collisions, const double z);
}  // namespace behavior_velocity_planner::dynamic_obstacle_stop::debug

#endif  // DEBUG_HPP_
