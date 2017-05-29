// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

#include "overlay_menu_display.h"

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreTechnique.h>

#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>

namespace jsk_rviz_plugins
{

  const int menu_padding_x = 100;
  const int menu_padding_y = 5;
  const int menu_last_padding_y = 30;
  const double animate_duration = 0.2;
  OverlayMenuDisplay::OverlayMenuDisplay() : Display()
  {
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      ros::message_traits::datatype<jsk_rviz_plugins::OverlayMenu>(),
      "jsk_rviz_plugins::OverlayMenu topic to subscribe to.",
      this, SLOT( updateTopic() ));
 
  }

  OverlayMenuDisplay::~OverlayMenuDisplay()
  {
    onDisable();
    delete update_topic_property_;
  }

  void OverlayMenuDisplay::onInitialize()
  {
    require_update_texture_ = false;
    animation_state_ = CLOSED;
  }
  
  void OverlayMenuDisplay::onEnable()
  {
    if (overlay_) {
      overlay_->show();
    }
    subscribe();
  }
  void OverlayMenuDisplay::onDisable()
  {
    if (overlay_) {
      overlay_->hide();
    }
    unsubscribe();
  }

  void OverlayMenuDisplay::unsubscribe()
  {
    sub_.shutdown();
  }

  void OverlayMenuDisplay::subscribe()
  {
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      sub_ = ros::NodeHandle().subscribe(topic_name, 1,
                                         &OverlayMenuDisplay::processMessage,
                                         this);
    }
  }

  void OverlayMenuDisplay::processMessage
  (const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg)
  {
    next_menu_ = msg;
  }

  bool OverlayMenuDisplay::isNeedToResize()
  {
    if (!current_menu_ && next_menu_) { // first time
      ROS_DEBUG("need to resize because this is the first time to draw");
      return true;
    }
    else if (!current_menu_ && !next_menu_) {
      // both are null, it means that ...
      // the plugin tries to draw without message reception
      ROS_DEBUG("no need to resize because the plugin tries to draw without message reception");
      return false;
    }
    else if (current_menu_ && !next_menu_) {
      // this is unexpected case
      ROS_DEBUG("no need to resize, this is unexpected case. please debug");
      return false;
    }
    else {
      if (current_menu_->menus.size() != next_menu_->menus.size()) {
        ROS_DEBUG("need to resize because the length of menu is different");
        return true;
      }
      else if (current_menu_->title != next_menu_->title) {
        return true;
      }
      else {
        // check all the menu is same or not
        for (size_t i = 0; i < current_menu_->menus.size(); i++) {
          if (current_menu_->menus[i] != next_menu_->menus[i]) {
            ROS_DEBUG("need to resize because the content of menu is different");
            return true;
          }
        }
        ROS_DEBUG("no need to resize because the content of menu is same");
        return false;
      }
    }
  }

  QFont OverlayMenuDisplay::font()
  {
    QFont font;
    font.setPointSize(20);
    return font;
  }
  
  QFontMetrics OverlayMenuDisplay::fontMetrics()
  {
    QFontMetrics fm(font());
    return fm;
  }
  
  int OverlayMenuDisplay::drawAreaWidth(
    const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg)
  {
    QFontMetrics fm = fontMetrics();
    int max_width = 0;
    for (size_t i = 0; i < msg->menus.size(); i++) {
      int w = fm.width(getMenuString(msg, i).c_str());
      if (max_width < w) {
        max_width = w;
      }
    }
    int w = fm.width(msg->title.c_str());
    
    if (max_width < w) {
      max_width = w;
    }
    return max_width + menu_padding_x * 2;
  }

  int OverlayMenuDisplay::drawAreaHeight(
    const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg)
  {
    QFontMetrics fm = fontMetrics();
    return fm.height() * (msg->menus.size() + 1)
      + menu_padding_y * (msg->menus.size() + 1 - 1)
      + menu_last_padding_y * 2;
  }
  
  void OverlayMenuDisplay::update(float wall_dt, float ros_dt)
  {
    if (!next_menu_) {
      ROS_DEBUG("next_menu_ is null, no need to update");
      return;
    }
    if (next_menu_->action == jsk_rviz_plugins::OverlayMenu::ACTION_CLOSE &&
        animation_state_ == CLOSED) {
      ROS_DEBUG("request is close and state is closed, we ignore it completely");
      return;
    }

    if (next_menu_->action == jsk_rviz_plugins::OverlayMenu::ACTION_CLOSE) {
      // need to close...
      if (animation_state_ == CLOSED) {
        // do nothing, it should be ignored above if sentence
        ROS_WARN("request is CLOSE and state is CLOSED, it should be ignored before...");
      }
      else if (animation_state_ == OPENED) { // OPENED -> CLOSING
        animation_state_ = CLOSING;
        animation_t_ = animate_duration;
      }
      else if (animation_state_ == CLOSING) {
        animation_t_ -= wall_dt;
        if (animation_t_ > 0) { // CLOSING -> CLOSING
          openingAnimation();
        }
        else { // CLOSING -> CLOSED
          animation_t_ = 0;
          openingAnimation();
          animation_state_ = CLOSED;
        }
      }
      else if (animation_state_ == OPENING) { // if the status is OPENING, we open it anyway...??
        animation_t_ += wall_dt;
        if (animation_t_ < animate_duration) { // OPENING -> OPENING
          openingAnimation();
        }
        else {                  // OPENING -> OPENED
          redraw();
          animation_state_ = OPENED;
        }
      }
    }
    else {                      // OPEN request
      if (animation_state_ == CLOSED) { // CLOSED -> OPENING, do nothing just change the state
        animation_t_ = 0.0;
        animation_state_ = OPENING;
      }
      else if (animation_state_ == OPENING) {
        animation_t_ += wall_dt;
        ROS_DEBUG("animation_t: %f", animation_t_);
        if (animation_t_ < animate_duration) { // OPENING -> OPENING
          openingAnimation();
        }
        else {                  // OPENING -> OPENED
          redraw();
          animation_state_ = OPENED;
        }
      }
      else if (animation_state_ == OPENED) { // OPENED -> OPENED
        if (isNeedToRedraw()) {
          redraw();
        }
      }
      else if (animation_state_ == CLOSING) { // CLOSING, we close it anyway...
        animation_t_ -= wall_dt;
        if (animation_t_ > 0) {
          openingAnimation();
        }
        else {
          animation_t_ = 0;
          openingAnimation();
          animation_state_ = CLOSED;
        }
      }
    }
    //redraw();
    //current_menu_ = next_menu_;
  }

  bool OverlayMenuDisplay::isNeedToRedraw() {
    return true;
  }

  std::string OverlayMenuDisplay::getMenuString(
    const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg,
    size_t index)
  {
    if (index >= msg->menus.size()) {
      return "";
    }
    else {
      return msg->menus[index];
    }
  }

  void OverlayMenuDisplay::prepareOverlay()
  {
    if (!overlay_) {
      static int count = 0;
      rviz::UniformStringStream ss;
      ss << "OverlayMenuDisplayObject" << count++;
      overlay_.reset(new OverlayObject(ss.str()));
      overlay_->show();
    }
    if (!overlay_->isTextureReady() || isNeedToResize()) {
      overlay_->updateTextureSize(drawAreaWidth(next_menu_), drawAreaHeight(next_menu_));
    }
    else {
      ROS_DEBUG("no need to update texture size");
    }
  }
  
  void OverlayMenuDisplay::openingAnimation()
  {
    ROS_DEBUG("openningAnimation");
    prepareOverlay();
    int current_width = animation_t_ / animate_duration * overlay_->getTextureWidth();
    int current_height = animation_t_ / animate_duration * overlay_->getTextureHeight();
    {
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QColor bg_color(0, 0, 0, 255.0 / 2.0);
      QColor transparent(0, 0, 0, 0.0);
      QImage Hud = buffer.getQImage(*overlay_);
      for (int i = 0; i < overlay_->getTextureWidth(); i++) {
        for (int j = 0; j < overlay_->getTextureHeight(); j++) {
          if (i > (overlay_->getTextureWidth() - current_width) / 2.0 &&
              i < overlay_->getTextureWidth() - (overlay_->getTextureWidth() - current_width) / 2.0 &&
              j > (overlay_->getTextureHeight() - current_height) / 2.0 &&
              j < overlay_->getTextureHeight() - (overlay_->getTextureHeight() - current_height) / 2.0) {
            Hud.setPixel(i, j, bg_color.rgba());
          }
          else {
            Hud.setPixel(i, j, transparent.rgba());
          }
        }
      }
    }
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    int window_width = context_->getViewManager()->getRenderPanel()->width();
    int window_height = context_->getViewManager()->getRenderPanel()->height();
    double window_left = (window_width - (int)overlay_->getTextureWidth()) / 2.0;
    double window_top = (window_height - (int)overlay_->getTextureHeight()) / 2.0;
    overlay_->setPosition(window_left, window_top);
                          
    current_menu_ = next_menu_;
  }
  
  void OverlayMenuDisplay::redraw()
  {
    ROS_DEBUG("redraw");
    prepareOverlay();
    {
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QColor bg_color(0, 0, 0, 255.0 / 2.0);
      QColor fg_color(25, 255, 240, 255.0);
      QImage Hud = buffer.getQImage(*overlay_, bg_color);
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg_color, 1, Qt::SolidLine));
      painter.setFont(font());
      int line_height = fontMetrics().height();
      int w = drawAreaWidth(next_menu_);
      painter.drawText(menu_padding_x,  menu_padding_y,
                       w, line_height,
                       Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                       next_menu_->title.c_str());
      for (size_t i = 0; i < next_menu_->menus.size(); i++) {
        std::string menu = getMenuString(next_menu_, i);
        painter.drawText(menu_padding_x, line_height * ( 1 + i ) + menu_padding_y + menu_last_padding_y,
                         w, line_height,
                         Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                         menu.c_str());
      }
      if (next_menu_->current_index <= next_menu_->menus.size()) {
        // draw '>'
        painter.drawText(menu_padding_x - fontMetrics().width(">") * 2,
                         line_height * ( 1 + next_menu_->current_index ) + menu_padding_y + menu_last_padding_y,
                         w, line_height,
                         Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                         ">");
      }
      // draw line
      int texture_width = overlay_->getTextureWidth();
      int texture_height = overlay_->getTextureHeight();
      painter.drawLine(menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                       menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
      painter.drawLine(texture_width - menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                       texture_width - menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
      painter.drawLine(menu_padding_x / 2, menu_last_padding_y / 2 + line_height,
                       texture_width - menu_padding_x / 2, menu_last_padding_y / 2 + line_height);
      painter.drawLine(menu_padding_x / 2, texture_height - menu_last_padding_y / 2,
                       texture_width - menu_padding_x / 2, texture_height - menu_last_padding_y / 2);
      
      painter.end();
      current_menu_ = next_menu_;
    }
    overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
    int window_width = context_->getViewManager()->getRenderPanel()->width();
    int window_height = context_->getViewManager()->getRenderPanel()->height();
    double window_left = (window_width - (int)overlay_->getTextureWidth()) / 2.0;
    double window_top = (window_height - (int)overlay_->getTextureHeight()) / 2.0;
    overlay_->setPosition(window_left, window_top);
  }
  
  void OverlayMenuDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::OverlayMenuDisplay, rviz::Display )
