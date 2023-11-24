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

#ifndef OBJECTS_OF_INTEREST_MARKER_INTERFACE__COLORING_HPP_
#define OBJECTS_OF_INTEREST_MARKER_INTERFACE__COLORING_HPP_
#include "objects_of_interest_marker_interface/marker_data.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <std_msgs/msg/color_rgba.hpp>

namespace objects_of_interest_marker_interface::coloring
{
std_msgs::msg::ColorRGBA getGreen(const float alpha);
std_msgs::msg::ColorRGBA getAmber(const float alpha);
std_msgs::msg::ColorRGBA getRed(const float alpha);
std_msgs::msg::ColorRGBA getGray(const float alpha);
}  // namespace objects_of_interest_marker_interface::coloring

#endif  // OBJECTS_OF_INTEREST_MARKER_INTERFACE__COLORING_HPP_
