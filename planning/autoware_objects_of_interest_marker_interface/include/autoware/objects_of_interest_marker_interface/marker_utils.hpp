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

#ifndef AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_UTILS_HPP_
#define AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_UTILS_HPP_
#include "autoware/objects_of_interest_marker_interface/coloring.hpp"
#include "autoware/objects_of_interest_marker_interface/marker_data.hpp"

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/constants.hpp>
#include <autoware/universe_utils/math/trigonometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

namespace autoware::objects_of_interest_marker_interface::marker_utils
{
/**
 * @brief Create arrow marker from object marker data
 * @param id Marker id
 * @param data Object marker data
 * @param name Module name
 * @param height_offset Height offset of arrow marker
 * @param arrow_length Length of arrow marker
 */
visualization_msgs::msg::Marker createArrowMarker(
  const size_t id, const ObjectMarkerData & data, const std::string & name,
  const double height_offset, const double arrow_length = 1.0);

/**
 * @brief Create circle marker from object marker data
 * @param id Marker id
 * @param data Object marker data
 * @param name Module name
 * @param radius Radius of circle marker
 * @param height_offset Height offset of circle marker
 * @param line_width Line width of circle marker
 */
visualization_msgs::msg::Marker createCircleMarker(
  const size_t id, const ObjectMarkerData & data, const std::string & name, const double radius,
  const double height_offset, const double line_width = 0.1);

/**
 * @brief Create text marker visualizing module name
 * @param id Marker id
 * @param data Object marker data
 * @param name Module name
 * @param height_offset Height offset of target marker
 * @param text_size Text size
 */
visualization_msgs::msg::Marker createNameTextMarker(
  const size_t id, const ObjectMarkerData & data, const std::string & name,
  const double height_offset, const double text_size = 0.5);

/**
 * @brief Create target marker from object marker data
 * @param id Marker id
 * @param data Object marker data
 * @param name Module name
 * @param height_offset Height offset of target marker
 * @param arrow_length Length of arrow marker
 * @param line_width Line width of circle marker
 */
visualization_msgs::msg::MarkerArray createTargetMarker(
  const size_t id, const ObjectMarkerData & data, const std::string & name,
  const double height_offset, const double arrow_length = 1.0, const double line_width = 0.1);
}  // namespace autoware::objects_of_interest_marker_interface::marker_utils

#endif  // AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_UTILS_HPP_
