// Copyright 2024 The Autoware Contributors
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

// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Team Spatzenhirn
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef OVERLAY_TEXT_DISPLAY_HPP_
#define OVERLAY_TEXT_DISPLAY_HPP_

#include "autoware_overlay_msgs/msg/overlay_text.hpp"
#ifndef Q_MOC_RUN
#include "overlay_utils.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <OgreColourValue.h>
#include <OgreMaterial.h>
#include <std_msgs/msg/color_rgba.h>

#include <string>
#endif

namespace autoware_overlay_rviz_plugin
{
class OverlayTextDisplay
: public rviz_common::RosTopicDisplay<autoware_overlay_msgs::msg::OverlayText>
{
  Q_OBJECT
public:
  OverlayTextDisplay();
  virtual ~OverlayTextDisplay();

protected:
  autoware_overlay_rviz_plugin::OverlayObject::SharedPtr overlay_;

  int texture_width_;
  int texture_height_;

  bool overtake_fg_color_properties_;
  bool overtake_bg_color_properties_;
  bool overtake_position_properties_;
  bool align_bottom_;
  bool invert_shadow_;
  QColor bg_color_;
  QColor fg_color_;
  int text_size_;
  int line_width_;
  std::string text_;
  QStringList font_families_;
  std::string font_;
  int horizontal_dist_;
  int vertical_dist_;
  HorizontalAlignment horizontal_alignment_;
  VerticalAlignment vertical_alignment_;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  bool require_update_texture_;
  // properties are raw pointers since they are owned by Qt
  rviz_common::properties::BoolProperty * overtake_position_properties_property_;
  rviz_common::properties::BoolProperty * overtake_fg_color_properties_property_;
  rviz_common::properties::BoolProperty * overtake_bg_color_properties_property_;
  rviz_common::properties::BoolProperty * align_bottom_property_;
  rviz_common::properties::BoolProperty * invert_shadow_property_;
  rviz_common::properties::IntProperty * hor_dist_property_;
  rviz_common::properties::IntProperty * ver_dist_property_;
  rviz_common::properties::EnumProperty * hor_alignment_property_;
  rviz_common::properties::EnumProperty * ver_alignment_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * height_property_;
  rviz_common::properties::IntProperty * text_size_property_;
  rviz_common::properties::IntProperty * line_width_property_;
  rviz_common::properties::ColorProperty * bg_color_property_;
  rviz_common::properties::FloatProperty * bg_alpha_property_;
  rviz_common::properties::ColorProperty * fg_color_property_;
  rviz_common::properties::FloatProperty * fg_alpha_property_;
  rviz_common::properties::EnumProperty * font_property_;

protected Q_SLOTS:
  void updateOvertakePositionProperties();
  void updateOvertakeFGColorProperties();
  void updateOvertakeBGColorProperties();
  void updateAlignBottom();
  void updateInvertShadow();
  void updateHorizontalDistance();
  void updateVerticalDistance();
  void updateHorizontalAlignment();
  void updateVerticalAlignment();
  void updateWidth();
  void updateHeight();
  void updateTextSize();
  void updateFGColor();
  void updateFGAlpha();
  void updateBGColor();
  void updateBGAlpha();
  void updateFont();
  void updateLineWidth();

private:
  void processMessage(autoware_overlay_msgs::msg::OverlayText::ConstSharedPtr msg) override;
};
}  // namespace autoware_overlay_rviz_plugin

#endif  // OVERLAY_TEXT_DISPLAY_HPP_
