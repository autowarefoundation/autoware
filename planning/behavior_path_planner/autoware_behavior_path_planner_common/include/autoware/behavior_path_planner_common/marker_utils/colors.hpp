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
#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__MARKER_UTILS__COLORS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__MARKER_UTILS__COLORS_HPP_

#include "autoware/universe_utils/ros/marker_helper.hpp"

#include "std_msgs/msg/detail/color_rgba__struct.hpp"

#include <vector>

namespace marker_utils::colors
{
using std_msgs::msg::ColorRGBA;

inline ColorRGBA red(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(1., 0., 0., a);
}

inline ColorRGBA green(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(0., 1., 0., a);
}

inline ColorRGBA blue(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(0., 0., 1., a);
}

inline ColorRGBA yellow(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(1., 1., 0., a);
}

inline ColorRGBA aqua(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(0., 1., 1., a);
}

inline ColorRGBA magenta(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(1., 0., 1., a);
}

inline ColorRGBA medium_orchid(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(0.729, 0.333, 0.827, a);
}

inline ColorRGBA light_pink(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(1., 0.713, 0.756, a);
}

inline ColorRGBA light_yellow(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(1., 1., 0.878, a);
}

inline ColorRGBA light_steel_blue(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(0.690, 0.768, 0.870, a);
}

inline ColorRGBA white(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(1., 1., 1., a);
}

inline ColorRGBA grey(float a = 0.99)
{
  return autoware::universe_utils::createMarkerColor(.5, .5, .5, a);
}

inline std::vector<ColorRGBA> colors_list(float a = 0.99)
{
  return {red(a),     green(a),         blue(a),       yellow(a),       aqua(a),
          magenta(a), medium_orchid(a), light_pink(a), light_yellow(a), light_steel_blue(a)};
}
}  // namespace marker_utils::colors

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__MARKER_UTILS__COLORS_HPP_
