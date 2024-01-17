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

// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Software License Agreement (BSD License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.S SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#include "tier4_debug_rviz_plugin/string_stamped.hpp"

#include "tier4_debug_rviz_plugin/jsk_overlay_utils.hpp"

#include <QPainter>
#include <rviz_common/uniform_string_stream.hpp>

#include <X11/Xlib.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace rviz_plugins
{
StringStampedOverlayDisplay::StringStampedOverlayDisplay()
{
  const Screen * screen_info = DefaultScreenOfDisplay(XOpenDisplay(NULL));

  constexpr float hight_4k = 2160.0;
  const float scale = static_cast<float>(screen_info->height) / hight_4k;
  const auto left = static_cast<int>(std::round(1024 * scale));
  const auto top = static_cast<int>(std::round(128 * scale));

  property_text_color_ = new rviz_common::properties::ColorProperty(
    "Text Color", QColor(25, 255, 240), "text color", this, SLOT(updateVisualization()), this);
  property_left_ = new rviz_common::properties::IntProperty(
    "Left", left, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty(
    "Top", top, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_value_height_offset_ = new rviz_common::properties::IntProperty(
    "Value height offset", 0, "Height offset of the plotter window", this,
    SLOT(updateVisualization()));
  property_font_size_ = new rviz_common::properties::IntProperty(
    "Font Size", 15, "Font Size", this, SLOT(updateVisualization()), this);
  property_font_size_->setMin(1);
  property_max_letter_num_ = new rviz_common::properties::IntProperty(
    "Max Letter Num", 100, "Max Letter Num", this, SLOT(updateVisualization()), this);
  property_max_letter_num_->setMin(10);
}

StringStampedOverlayDisplay::~StringStampedOverlayDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void StringStampedOverlayDisplay::onInitialize()
{
  RTDClass::onInitialize();

  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "StringOverlayDisplayObject" << count++;
  auto logger = context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger();
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(scene_manager_, logger, ss.str()));

  overlay_->show();

  const int texture_size = property_font_size_->getInt() * property_max_letter_num_->getInt();
  overlay_->updateTextureSize(texture_size, texture_size);
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
}

void StringStampedOverlayDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void StringStampedOverlayDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void StringStampedOverlayDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::lock_guard<std::mutex> message_lock(mutex_);
  if (!last_msg_ptr_) {
    return;
  }

  // Display
  QColor background_color;
  background_color.setAlpha(0);
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);
  hud.fill(background_color);

  QPainter painter(&hud);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const int w = overlay_->getTextureWidth() - line_width_;
  const int h = overlay_->getTextureHeight() - line_width_;

  // text
  QColor text_color(property_text_color_->getColor());
  text_color.setAlpha(255);
  painter.setPen(QPen(text_color, static_cast<int>(2), Qt::SolidLine));
  QFont font = painter.font();
  font.setPixelSize(property_font_size_->getInt());
  font.setBold(true);
  painter.setFont(font);

  // same as above, but align on right side
  painter.drawText(
    0, std::min(property_value_height_offset_->getInt(), h - 1), w,
    std::max(h - property_value_height_offset_->getInt(), 1), Qt::AlignLeft | Qt::AlignTop,
    last_msg_ptr_->data.c_str());
  painter.end();
  updateVisualization();
}

void StringStampedOverlayDisplay::processMessage(
  const tier4_debug_msgs::msg::StringStamped::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) {
    return;
  }

  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    last_msg_ptr_ = msg_ptr;
  }

  queueRender();
}

void StringStampedOverlayDisplay::updateVisualization()
{
  const int texture_size = property_font_size_->getInt() * property_max_letter_num_->getInt();
  overlay_->updateTextureSize(texture_size, texture_size);
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::StringStampedOverlayDisplay, rviz_common::Display)
