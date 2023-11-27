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

#include "objects_of_interest_marker_interface/marker_utils.hpp"

namespace objects_of_interest_marker_interface::marker_utils
{
using geometry_msgs::msg::Point;

using std_msgs::msg::ColorRGBA;

using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerScale;

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

Marker createArrowMarker(
  const size_t id, const ObjectMarkerData & data, const std::string & name,
  const double height_offset, const double arrow_length)
{
  const double line_width = 0.25 * arrow_length;
  const double head_width = 0.5 * arrow_length;
  const double head_height = 0.5 * arrow_length;

  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name + "_arrow", id, Marker::ARROW,
    createMarkerScale(line_width, head_width, head_height), data.color);

  const double height = 0.5 * data.shape.dimensions.z;

  Point src, dst;
  src = data.pose.position;
  src.z += height + height_offset + arrow_length;
  dst = data.pose.position;
  dst.z += height + height_offset;

  marker.points.push_back(src);
  marker.points.push_back(dst);

  return marker;
}

Marker createCircleMarker(
  const size_t id, const ObjectMarkerData & data, const std::string & name, const double radius,
  const double height_offset, const double line_width)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name, id, Marker::LINE_STRIP,
    createMarkerScale(line_width, 0.0, 0.0), data.color);

  const double height = 0.5 * data.shape.dimensions.z;

  constexpr size_t num_points = 20;
  for (size_t i = 0; i < num_points; ++i) {
    Point point;
    const double ratio = static_cast<double>(i) / static_cast<double>(num_points);
    const double theta = 2 * tier4_autoware_utils::pi * ratio;
    point.x = data.pose.position.x + radius * tier4_autoware_utils::cos(theta);
    point.y = data.pose.position.y + radius * tier4_autoware_utils::sin(theta);
    point.z = data.pose.position.z + height + height_offset;
    marker.points.push_back(point);
  }
  marker.points.push_back(marker.points.front());

  return marker;
}

visualization_msgs::msg::Marker createNameTextMarker(
  const size_t id, const ObjectMarkerData & data, const std::string & name,
  const double height_offset, const double text_size)
{
  Marker marker = createDefaultMarker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name + "_name_text", id, Marker::TEXT_VIEW_FACING,
    createMarkerScale(0.0, 0.0, text_size), coloring::getGray(data.color.a));

  marker.text = name;

  const double height = 0.5 * data.shape.dimensions.z;
  marker.pose = data.pose;
  marker.pose.position.z += height + height_offset;

  return marker;
}

MarkerArray createTargetMarker(
  const size_t id, const ObjectMarkerData & data, const std::string & name,
  const double height_offset, const double arrow_length, const double line_width)
{
  MarkerArray marker_array;
  marker_array.markers.push_back(createArrowMarker(id, data, name, height_offset, arrow_length));
  marker_array.markers.push_back(createCircleMarker(
    id, data, name + "_circle1", 0.5 * arrow_length, height_offset + 0.75 * arrow_length,
    line_width));
  marker_array.markers.push_back(createCircleMarker(
    id, data, name + "_circle2", 0.75 * arrow_length, height_offset + 0.75 * arrow_length,
    line_width));
  marker_array.markers.push_back(
    createNameTextMarker(id, data, name, height_offset + 1.5 * arrow_length, 0.5 * arrow_length));

  return marker_array;
}
}  // namespace objects_of_interest_marker_interface::marker_utils
