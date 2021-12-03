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
#ifndef OBSTACLE_AVOIDANCE_PLANNER__MARKER_HELPER_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__MARKER_HELPER_HPP_

#include <visualization_msgs/msg/marker_array.hpp>

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

inline void appendMarkerArray(
  const visualization_msgs::msg::MarkerArray & additional_marker_array,
  visualization_msgs::msg::MarkerArray * marker_array)
{
  for (const auto & marker : additional_marker_array.markers) {
    marker_array->markers.push_back(marker);
  }
}
#endif  // OBSTACLE_AVOIDANCE_PLANNER__MARKER_HELPER_HPP_
