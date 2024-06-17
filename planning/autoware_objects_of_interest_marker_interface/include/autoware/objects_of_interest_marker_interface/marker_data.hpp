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

#ifndef AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_DATA_HPP_
#define AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_DATA_HPP_

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace autoware::objects_of_interest_marker_interface
{
struct ObjectMarkerData
{
  geometry_msgs::msg::Pose pose{};
  autoware_perception_msgs::msg::Shape shape{};
  std_msgs::msg::ColorRGBA color;
};

enum class ColorName { GRAY, GREEN, AMBER, RED };
}  // namespace autoware::objects_of_interest_marker_interface

#endif  // AUTOWARE__OBJECTS_OF_INTEREST_MARKER_INTERFACE__MARKER_DATA_HPP_
