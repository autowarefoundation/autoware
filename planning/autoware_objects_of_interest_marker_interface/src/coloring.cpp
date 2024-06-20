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

#include "autoware/objects_of_interest_marker_interface/coloring.hpp"

namespace
{
std_msgs::msg::ColorRGBA convertFromColorCode(const uint64_t code, const float alpha)
{
  const float r = static_cast<int>(code >> 16) / 255.0;
  const float g = static_cast<int>((code << 48) >> 56) / 255.0;
  const float b = static_cast<int>((code << 56) >> 56) / 255.0;

  return autoware::universe_utils::createMarkerColor(r, g, b, alpha);
}
}  // namespace

namespace autoware::objects_of_interest_marker_interface::coloring
{
std_msgs::msg::ColorRGBA getGreen(const float alpha)
{
  constexpr uint64_t code = 0x00e676;
  return convertFromColorCode(code, alpha);
}

std_msgs::msg::ColorRGBA getAmber(const float alpha)
{
  constexpr uint64_t code = 0xffea00;
  return convertFromColorCode(code, alpha);
}

std_msgs::msg::ColorRGBA getRed(const float alpha)
{
  constexpr uint64_t code = 0xff3d00;
  return convertFromColorCode(code, alpha);
}

std_msgs::msg::ColorRGBA getGray(const float alpha)
{
  constexpr uint64_t code = 0xbdbdbd;
  return convertFromColorCode(code, alpha);
}
}  // namespace autoware::objects_of_interest_marker_interface::coloring
