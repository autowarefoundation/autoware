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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__MARKER_HELPER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__MARKER_HELPER_HPP_

#include <rclcpp/time.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <optional>
#include <string>

namespace autoware::universe_utils
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

visualization_msgs::msg::Marker createDefaultMarker(
  const std::string & frame_id, const rclcpp::Time & now, const std::string & ns, const int32_t id,
  const int32_t type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

visualization_msgs::msg::Marker createDeletedDefaultMarker(
  const rclcpp::Time & now, const std::string & ns, const int32_t id);

void appendMarkerArray(
  const visualization_msgs::msg::MarkerArray & additional_marker_array,
  visualization_msgs::msg::MarkerArray * marker_array,
  const std::optional<rclcpp::Time> & current_time = {});

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__MARKER_HELPER_HPP_
