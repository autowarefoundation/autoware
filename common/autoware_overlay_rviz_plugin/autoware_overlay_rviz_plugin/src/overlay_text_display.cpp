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

#include "overlay_text_display.hpp"

#include <QFontDatabase>
#include <QPainter>
#include <QStaticText>
#include <QTextDocument>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/render_system.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTexture.h>

#include <regex>
#include <sstream>

namespace autoware_overlay_rviz_plugin
{
OverlayTextDisplay::OverlayTextDisplay()
: texture_width_(0),
  texture_height_(0),
  bg_color_(0, 0, 0, 0),
  fg_color_(255, 255, 255, 255.0),
  text_size_(14),
  line_width_(2),
  text_(""),
  font_(""),
  require_update_texture_(false)
{
  overtake_position_properties_property_ = new rviz_common::properties::BoolProperty(
    "Overtake Position Properties", false,
    "overtake position properties specified by message such as left, top and font", this,
    SLOT(updateOvertakePositionProperties()));
  overtake_fg_color_properties_property_ = new rviz_common::properties::BoolProperty(
    "Overtake FG Color Properties", false,
    "overtake color properties specified by message such as foreground color and alpha", this,
    SLOT(updateOvertakeFGColorProperties()));
  overtake_bg_color_properties_property_ = new rviz_common::properties::BoolProperty(
    "Overtake BG Color Properties", false,
    "overtake color properties specified by message such as background color and alpha", this,
    SLOT(updateOvertakeBGColorProperties()));
  align_bottom_property_ = new rviz_common::properties::BoolProperty(
    "Align Bottom", false, "align text with the bottom of the overlay region", this,
    SLOT(updateAlignBottom()));
  invert_shadow_property_ = new rviz_common::properties::BoolProperty(
    "Invert Shadow", false, "make shadow lighter than original text", this,
    SLOT(updateInvertShadow()));
  hor_dist_property_ = new rviz_common::properties::IntProperty(
    "hor_dist", 0, "horizontal distance to anchor", this, SLOT(updateHorizontalDistance()));
  ver_dist_property_ = new rviz_common::properties::IntProperty(
    "ver_dist", 0, "vertical distance to anchor", this, SLOT(updateVerticalDistance()));
  hor_alignment_property_ = new rviz_common::properties::EnumProperty(
    "hor_alignment", "left", "horizontal alignment of the overlay", this,
    SLOT(updateHorizontalAlignment()));
  hor_alignment_property_->addOption("left", autoware_overlay_msgs::msg::OverlayText::LEFT);
  hor_alignment_property_->addOption("center", autoware_overlay_msgs::msg::OverlayText::CENTER);
  hor_alignment_property_->addOption("right", autoware_overlay_msgs::msg::OverlayText::RIGHT);
  ver_alignment_property_ = new rviz_common::properties::EnumProperty(
    "ver_alignment", "top", "vertical alignment of the overlay", this,
    SLOT(updateVerticalAlignment()));
  ver_alignment_property_->addOption("top", autoware_overlay_msgs::msg::OverlayText::TOP);
  ver_alignment_property_->addOption("center", autoware_overlay_msgs::msg::OverlayText::CENTER);
  ver_alignment_property_->addOption("bottom", autoware_overlay_msgs::msg::OverlayText::BOTTOM);
  width_property_ = new rviz_common::properties::IntProperty(
    "width", 128, "width position", this, SLOT(updateWidth()));
  width_property_->setMin(0);
  height_property_ = new rviz_common::properties::IntProperty(
    "height", 128, "height position", this, SLOT(updateHeight()));
  height_property_->setMin(0);
  text_size_property_ = new rviz_common::properties::IntProperty(
    "text size", 12, "text size", this, SLOT(updateTextSize()));
  text_size_property_->setMin(0);
  line_width_property_ = new rviz_common::properties::IntProperty(
    "line width", 2, "line width", this, SLOT(updateLineWidth()));
  line_width_property_->setMin(0);
  fg_color_property_ = new rviz_common::properties::ColorProperty(
    "Foreground Color", QColor(25, 255, 240), "Foreground Color", this, SLOT(updateFGColor()));
  fg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Foreground Alpha", 0.8, "Foreground Alpha", this, SLOT(updateFGAlpha()));
  fg_alpha_property_->setMin(0.0);
  fg_alpha_property_->setMax(1.0);
  bg_color_property_ = new rviz_common::properties::ColorProperty(
    "Background Color", QColor(0, 0, 0), "Background Color", this, SLOT(updateBGColor()));
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", 0.8, "Background Alpha", this, SLOT(updateBGAlpha()));
  bg_alpha_property_->setMin(0.0);
  bg_alpha_property_->setMax(1.0);

  QFontDatabase database;
  font_families_ = database.families();
  font_property_ = new rviz_common::properties::EnumProperty(
    "font", "DejaVu Sans Mono", "font", this, SLOT(updateFont()));
  for (ssize_t i = 0; i < font_families_.size(); i++) {
    font_property_->addOption(font_families_[i], static_cast<int>(i));
  }
}

OverlayTextDisplay::~OverlayTextDisplay()
{
  onDisable();
}

void OverlayTextDisplay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
  subscribe();
}

void OverlayTextDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
  unsubscribe();
}

// only the first time
void OverlayTextDisplay::onInitialize()
{
  RTDClass::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

  onEnable();
  updateTopic();
  updateOvertakePositionProperties();
  updateOvertakeFGColorProperties();
  updateOvertakeBGColorProperties();
  updateAlignBottom();
  updateInvertShadow();
  updateHorizontalDistance();
  updateVerticalDistance();
  updateHorizontalAlignment();
  updateVerticalAlignment();
  updateWidth();
  updateHeight();
  updateTextSize();
  updateFGColor();
  updateFGAlpha();
  updateBGColor();
  updateBGAlpha();
  updateFont();
  updateLineWidth();
  require_update_texture_ = true;
}

void OverlayTextDisplay::update(float /*wall_dt*/, float /*ros_dt*/)
{
  if (!require_update_texture_) {
    return;
  }
  if (!isEnabled()) {
    return;
  }
  if (!overlay_) {
    return;
  }

  overlay_->updateTextureSize(texture_width_, texture_height_);
  {
    autoware_overlay_rviz_plugin::ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_, bg_color_);
    QPainter painter(&Hud);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color_, std::max(line_width_, 1), Qt::SolidLine));
    uint16_t w = overlay_->getTextureWidth();
    uint16_t h = overlay_->getTextureHeight();

    // font
    if (text_size_ != 0) {
      // QFont font = painter.font();
      QFont font(font_.length() > 0 ? font_.c_str() : "Liberation Sans");
      font.setPointSize(text_size_);
      font.setBold(true);
      painter.setFont(font);
    }
    if (text_.length() > 0) {
      QColor shadow_color;
      if (invert_shadow_)
        shadow_color = Qt::white;  // fg_color_.lighter();
      else
        shadow_color = Qt::black;  // fg_color_.darker();
      shadow_color.setAlpha(fg_color_.alpha());

      std::string color_wrapped_text =
        (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>") % text_ %
         fg_color_.red() % fg_color_.green() % fg_color_.blue() % fg_color_.alpha())
          .str();

      // find a remove "color: XXX;" regex match to generate a proper shadow
      std::regex color_tag_re("color:.+?;");
      std::string null_char("");
      std::string formatted_text_ = std::regex_replace(text_, color_tag_re, null_char);
      std::string color_wrapped_shadow =
        (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>") %
         formatted_text_ % shadow_color.red() % shadow_color.green() % shadow_color.blue() %
         shadow_color.alpha())
          .str();

      QStaticText static_text(
        boost::algorithm::replace_all_copy(color_wrapped_text, "\n", "<br >").c_str());
      static_text.setTextWidth(w);

      painter.setPen(QPen(shadow_color, std::max(line_width_, 1), Qt::SolidLine));
      QStaticText static_shadow(
        boost::algorithm::replace_all_copy(color_wrapped_shadow, "\n", "<br >").c_str());
      static_shadow.setTextWidth(w);

      if (!align_bottom_) {
        painter.drawStaticText(1, 1, static_shadow);
        painter.drawStaticText(0, 0, static_text);
      } else {
        QStaticText only_wrapped_text(color_wrapped_text.c_str());
        QFontMetrics fm(painter.fontMetrics());
        QRect text_rect = fm.boundingRect(
          0, 0, w, h, Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
          only_wrapped_text.text().remove(QRegExp("<[^>]*>")));
        painter.drawStaticText(1, h - text_rect.height() + 1, static_shadow);
        painter.drawStaticText(0, h - text_rect.height(), static_text);
      }
    }
    painter.end();
  }
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
  require_update_texture_ = false;
}

void OverlayTextDisplay::reset()
{
  RTDClass::reset();

  if (overlay_) {
    overlay_->hide();
  }
}

void OverlayTextDisplay::processMessage(autoware_overlay_msgs::msg::OverlayText::ConstSharedPtr msg)
{
  if (!isEnabled()) {
    return;
  }
  if (!overlay_) {
    static int count = 0;
    std::stringstream ss;
    ss << "OverlayTextDisplayObject" << count++;
    overlay_.reset(new autoware_overlay_rviz_plugin::OverlayObject(ss.str()));
    overlay_->show();
  }
  if (overlay_) {
    if (msg->action == autoware_overlay_msgs::msg::OverlayText::DELETE) {
      overlay_->hide();
    } else if (msg->action == autoware_overlay_msgs::msg::OverlayText::ADD) {
      overlay_->show();
    }
  }

  // store message for update method
  text_ = msg->text;

  if (!overtake_position_properties_) {
    texture_width_ = msg->width;
    texture_height_ = msg->height;
    text_size_ = msg->text_size;
    horizontal_dist_ = msg->horizontal_distance;
    vertical_dist_ = msg->vertical_distance;

    horizontal_alignment_ = HorizontalAlignment{msg->horizontal_alignment};
    vertical_alignment_ = VerticalAlignment{msg->vertical_alignment};
  }
  if (!overtake_bg_color_properties_)
    bg_color_ = QColor(
      msg->bg_color.r * 255.0, msg->bg_color.g * 255.0, msg->bg_color.b * 255.0,
      msg->bg_color.a * 255.0);
  if (!overtake_fg_color_properties_) {
    fg_color_ = QColor(
      msg->fg_color.r * 255.0, msg->fg_color.g * 255.0, msg->fg_color.b * 255.0,
      msg->fg_color.a * 255.0);
    font_ = msg->font;
    line_width_ = msg->line_width;
  }
  if (overlay_) {
    overlay_->setPosition(
      horizontal_dist_, vertical_dist_, horizontal_alignment_, vertical_alignment_);
  }
  require_update_texture_ = true;
}

void OverlayTextDisplay::updateOvertakePositionProperties()
{
  if (!overtake_position_properties_ && overtake_position_properties_property_->getBool()) {
    updateVerticalDistance();
    updateHorizontalDistance();
    updateVerticalAlignment();
    updateHorizontalAlignment();
    updateWidth();
    updateHeight();
    updateTextSize();
    require_update_texture_ = true;
  }

  overtake_position_properties_ = overtake_position_properties_property_->getBool();
  if (overtake_position_properties_) {
    hor_dist_property_->show();
    ver_dist_property_->show();
    hor_alignment_property_->show();
    ver_alignment_property_->show();
    width_property_->show();
    height_property_->show();
    text_size_property_->show();
  } else {
    hor_dist_property_->hide();
    ver_dist_property_->hide();
    hor_alignment_property_->hide();
    ver_alignment_property_->hide();
    width_property_->hide();
    height_property_->hide();
    text_size_property_->hide();
  }
}

void OverlayTextDisplay::updateOvertakeFGColorProperties()
{
  if (!overtake_fg_color_properties_ && overtake_fg_color_properties_property_->getBool()) {
    // read all the parameters from properties
    updateFGColor();
    updateFGAlpha();
    updateFont();
    updateLineWidth();
    require_update_texture_ = true;
  }
  overtake_fg_color_properties_ = overtake_fg_color_properties_property_->getBool();
  if (overtake_fg_color_properties_) {
    fg_color_property_->show();
    fg_alpha_property_->show();
    line_width_property_->show();
    font_property_->show();
  } else {
    fg_color_property_->hide();
    fg_alpha_property_->hide();
    line_width_property_->hide();
    font_property_->hide();
  }
}

void OverlayTextDisplay::updateOvertakeBGColorProperties()
{
  if (!overtake_bg_color_properties_ && overtake_bg_color_properties_property_->getBool()) {
    // read all the parameters from properties
    updateBGColor();
    updateBGAlpha();
    require_update_texture_ = true;
  }
  overtake_bg_color_properties_ = overtake_bg_color_properties_property_->getBool();
  if (overtake_bg_color_properties_) {
    bg_color_property_->show();
    bg_alpha_property_->show();
  } else {
    bg_color_property_->hide();
    bg_alpha_property_->hide();
  }
}

void OverlayTextDisplay::updateAlignBottom()
{
  if (align_bottom_ != align_bottom_property_->getBool()) {
    require_update_texture_ = true;
  }
  align_bottom_ = align_bottom_property_->getBool();
}

void OverlayTextDisplay::updateInvertShadow()
{
  if (invert_shadow_ != invert_shadow_property_->getBool()) {
    require_update_texture_ = true;
  }
  invert_shadow_ = invert_shadow_property_->getBool();
}

void OverlayTextDisplay::updateVerticalDistance()
{
  vertical_dist_ = ver_dist_property_->getInt();
  if (overtake_position_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateHorizontalDistance()
{
  horizontal_dist_ = hor_dist_property_->getInt();
  if (overtake_position_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateVerticalAlignment()
{
  vertical_alignment_ =
    VerticalAlignment{static_cast<uint8_t>(ver_alignment_property_->getOptionInt())};

  if (overtake_position_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateHorizontalAlignment()
{
  horizontal_alignment_ =
    HorizontalAlignment{static_cast<uint8_t>(hor_alignment_property_->getOptionInt())};

  if (overtake_position_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateWidth()
{
  texture_width_ = width_property_->getInt();
  if (overtake_position_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateHeight()
{
  texture_height_ = height_property_->getInt();
  if (overtake_position_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateTextSize()
{
  text_size_ = text_size_property_->getInt();
  if (overtake_position_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateBGColor()
{
  QColor c = bg_color_property_->getColor();
  bg_color_.setRed(c.red());
  bg_color_.setGreen(c.green());
  bg_color_.setBlue(c.blue());
  if (overtake_bg_color_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateBGAlpha()
{
  bg_color_.setAlpha(bg_alpha_property_->getFloat() * 255.0);
  if (overtake_bg_color_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateFGColor()
{
  QColor c = fg_color_property_->getColor();
  fg_color_.setRed(c.red());
  fg_color_.setGreen(c.green());
  fg_color_.setBlue(c.blue());
  if (overtake_fg_color_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateFGAlpha()
{
  fg_color_.setAlpha(fg_alpha_property_->getFloat() * 255.0);
  if (overtake_fg_color_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateFont()
{
  int font_index = font_property_->getOptionInt();
  if (font_index < font_families_.size()) {
    font_ = font_families_[font_index].toStdString();
  } else {
    RVIZ_COMMON_LOG_ERROR_STREAM("Unexpected error at selecting font index " << font_index);
    return;
  }
  if (overtake_fg_color_properties_) {
    require_update_texture_ = true;
  }
}

void OverlayTextDisplay::updateLineWidth()
{
  line_width_ = line_width_property_->getInt();
  if (overtake_fg_color_properties_) {
    require_update_texture_ = true;
  }
}

}  // namespace autoware_overlay_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware_overlay_rviz_plugin::OverlayTextDisplay, rviz_common::Display)
