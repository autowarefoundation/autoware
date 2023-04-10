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

#ifndef SCENE_MODULE__OUT_OF_LANE__DEBUG_HPP_
#define SCENE_MODULE__OUT_OF_LANE__DEBUG_HPP_

#include "scene_module/out_of_lane/types.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>

#include <string>

namespace behavior_velocity_planner::out_of_lane::debug
{
/// @brief add footprint markers to the given marker array
/// @param [inout] debug_marker_array marker array
/// @param [in] footprints footprints to turn into markers
/// @param [in] z z value to use for the markers
void add_footprint_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::BasicPolygons2d & footprints, const double z);
/// @brief add footprint markers to the given marker array
/// @param [inout] debug_marker_array marker array
/// @param [in] current_footprint footprint to turn into a marker
/// @param [in] current_overlapped_lanelets lanelets to turn into markers
/// @param [in] z z value to use for the markers
void add_current_overlap_marker(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::BasicPolygon2d & current_footprint,
  const lanelet::ConstLanelets & current_overlapped_lanelets, const double z);
/// @brief add footprint markers to the given marker array
/// @param [inout] debug_marker_array marker array
/// @param [in] lanelets lanelets to turn into markers
/// @param [in] ns namespace of the markers
/// @param [in] color namespace of the markers
void add_lanelet_markers(
  visualization_msgs::msg::MarkerArray & debug_marker_array,
  const lanelet::ConstLanelets & lanelets, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color);
}  // namespace behavior_velocity_planner::out_of_lane::debug

#endif  // SCENE_MODULE__OUT_OF_LANE__DEBUG_HPP_
