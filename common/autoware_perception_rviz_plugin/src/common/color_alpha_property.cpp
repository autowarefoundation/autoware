// Copyright 2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "autoware_perception_rviz_plugin/common/color_alpha_property.hpp"

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace common
{
ColorAlphaProperty::ColorAlphaProperty(
  const QColor & color_default, const float alpha_default,
  rviz_common::properties::Property * parent_property)
: m_color_property("Color", color_default, "Set color value.", parent_property),
  m_alpha_property(
    "Alpha", alpha_default, "Set transparency value. Should be between 0  and 1.", parent_property)
{
  m_alpha_property.setMax(1.0F);
  m_alpha_property.setMin(0.0F);
}

ColorAlphaProperty::operator std_msgs::msg::ColorRGBA() const
{
  std_msgs::msg::ColorRGBA ret;
  ret.r = static_cast<float>(m_color_property.getColor().redF());
  ret.g = static_cast<float>(m_color_property.getColor().greenF());
  ret.b = static_cast<float>(m_color_property.getColor().blueF());
  ret.a = m_alpha_property.getFloat();

  return ret;
}

}  // namespace common
}  // namespace rviz_plugins
}  // namespace autoware
