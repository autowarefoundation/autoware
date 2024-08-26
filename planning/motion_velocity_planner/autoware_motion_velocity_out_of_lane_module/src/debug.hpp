// Copyright 2024 TIER IV, Inc.
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

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::motion_velocity_planner::out_of_lane::debug
{
visualization_msgs::msg::MarkerArray create_debug_marker_array(
  const EgoData & ego_data, const OutOfLaneData & out_of_lane_data,
  const autoware_perception_msgs::msg::PredictedObjects & objects, DebugData & debug_data);
motion_utils::VirtualWalls create_virtual_walls(
  const geometry_msgs::msg::Pose & pose, const bool stop, const PlannerParam & params);
}  // namespace autoware::motion_velocity_planner::out_of_lane::debug

#endif  // DEBUG_HPP_
