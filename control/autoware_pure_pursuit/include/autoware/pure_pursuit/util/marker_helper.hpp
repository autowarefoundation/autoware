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

#ifndef AUTOWARE__PURE_PURSUIT__UTIL__MARKER_HELPER_HPP_
#define AUTOWARE__PURE_PURSUIT__UTIL__MARKER_HELPER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

namespace autoware::pure_pursuit
{
inline geometry_msgs::msg::Point createMarkerPosition(double x, double y, double z)
{
  geometry_msgs::msg::Point point;

  point.x = x;
  point.y = y;
  point.z = z;

  return point;
}

inline geometry_msgs::msg::Quaternion createMarkerOrientation(
  double x, double y, double z, double w)
{
  geometry_msgs::msg::Quaternion quaternion;

  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;

  return quaternion;
}

inline geometry_msgs::msg::Vector3 createMarkerScale(double x, double y, double z)
{
  geometry_msgs::msg::Vector3 scale;

  scale.x = x;
  scale.y = y;
  scale.z = z;

  return scale;
}

inline std_msgs::msg::ColorRGBA createMarkerColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;

  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  return color;
}

inline visualization_msgs::msg::Marker createDefaultMarker(
  const std::string & frame_id, const std::string & ns, const int32_t id, const int32_t type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame_id;
  //  ToDo
  //  marker.header.stamp = rclcpp::Node::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0);

  marker.pose.position = createMarkerPosition(0.0, 0.0, 0.0);
  marker.pose.orientation = createMarkerOrientation(0.0, 0.0, 0.0, 1.0);
  marker.scale = scale;
  marker.color = color;
  marker.frame_locked = true;

  return marker;
}

inline void appendMarkerArray(
  const visualization_msgs::msg::MarkerArray & additional_marker_array,
  visualization_msgs::msg::MarkerArray * marker_array)
{
  for (const auto & marker : additional_marker_array.markers) {
    marker_array->markers.push_back(marker);
  }
}
}  // namespace autoware::pure_pursuit

#endif  // AUTOWARE__PURE_PURSUIT__UTIL__MARKER_HELPER_HPP_
