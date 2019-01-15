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

#include "plotter_2d_display.h"
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <QPainter>

namespace jsk_rviz_plugins
{
  Plotter2DDisplay::Plotter2DDisplay()
    : rviz::Display(), min_value_(0.0), max_value_(0.0)
  {
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      ros::message_traits::datatype<std_msgs::Float32>(),
      "std_msgs::Float32 topic to subscribe to.",
      this, SLOT(updateTopic()));
    show_value_property_ = new rviz::BoolProperty(
      "Show Value", true,
      "Show value on plotter",
      this, SLOT(updateShowValue()));
    buffer_length_property_ = new rviz::IntProperty(
      "Buffer length", 100,
      ros::message_traits::datatype<std_msgs::Float32>(),
      this, SLOT(updateBufferSize()));
    width_property_ = new rviz::IntProperty("width", 128,
                                            "width of the plotter window",
                                            this, SLOT(updateWidth()));
    width_property_->setMin(1);
    width_property_->setMax(2000);
    height_property_ = new rviz::IntProperty("height", 128,
                                             "height of the plotter window",
                                             this, SLOT(updateHeight()));
    height_property_->setMin(1);
    height_property_->setMax(2000);
    left_property_ = new rviz::IntProperty("left", 128,
                                           "left of the plotter window",
                                           this, SLOT(updateLeft()));
    left_property_->setMin(0);
    top_property_ = new rviz::IntProperty("top", 128,
                                          "top of the plotter window",
                                          this, SLOT(updateTop()));
    top_property_->setMin(0);
    auto_scale_property_ = new rviz::BoolProperty(
      "auto scale", true,
      "enable auto scale",
      this, SLOT(updateAutoScale()));
    max_value_property_ = new rviz::FloatProperty(
      "max value", 1.0,
      "max value, used only if auto scale is disabled",
      this, SLOT(updateMaxValue()));
    min_value_property_ = new rviz::FloatProperty(
      "min value", -1.0,
      "min value, used only if auto scale is disabled",
      this, SLOT(updateMinValue()));
    fg_color_property_ = new rviz::ColorProperty(
      "foreground color", QColor(25, 255, 240),
      "color to draw line",
      this, SLOT(updateFGColor()));
    fg_alpha_property_ = new rviz::FloatProperty(
      "foreground alpha", 0.7,
      "alpha belnding value for foreground",
      this, SLOT(updateFGAlpha()));
    fg_alpha_property_->setMin(0);
    fg_alpha_property_->setMax(1.0);
    bg_color_property_ = new rviz::ColorProperty(
      "background color", QColor(0, 0, 0),
      "background color",
      this, SLOT(updateBGColor()));
    bg_alpha_property_ = new rviz::FloatProperty(
      "backround alpha", 0.0,
      "alpha belnding value for background",
      this, SLOT(updateBGAlpha()));
    bg_alpha_property_->setMin(0);
    bg_alpha_property_->setMax(1.0);
    line_width_property_ = new rviz::IntProperty("linewidth", 1,
                                                 "linewidth of the plot",
                                                 this, SLOT(updateLineWidth()));
    line_width_property_->setMin(1);
    line_width_property_->setMax(1000);
    show_border_property_ = new rviz::BoolProperty(
      "border", true,
      "show border or not",
      this, SLOT(updateShowBorder()));
    text_size_property_ = new rviz::IntProperty("text size", 12,
                                                "text size of the caption",
                                                this, SLOT(updateTextSize()));
    text_size_property_->setMin(1);
    text_size_property_->setMax(1000);
    show_caption_property_ = new rviz::BoolProperty(
      "show caption", true,
      "show caption or not",
      this, SLOT(updateShowCaption()));
    update_interval_property_ = new rviz::FloatProperty(
      "update interval", 0.04,
      "update interval of the plotter",
      this, SLOT(updateUpdateInterval()));
    update_interval_property_->setMin(0.0);
    update_interval_property_->setMax(100);
    auto_color_change_property_
      = new rviz::BoolProperty("auto color change",
                               false,
                               "change the color automatically",
                               this, SLOT(updateAutoColorChange()));
    max_color_property_
      = new rviz::ColorProperty(
        "max color",
        QColor(255, 0, 0),
        "only used if auto color change is set to True.",
        this, SLOT(updateMaxColor()));
  }

  Plotter2DDisplay::~Plotter2DDisplay()
  {
    onDisable();
    // delete update_topic_property_;
    // delete buffer_length_property_;
    // delete fg_color_property_;
    // delete bg_color_property_;
    // delete fg_alpha_property_;
    // delete bg_alpha_property_;
    // delete top_property_;
    // delete left_property_;
    // delete width_property_;
    // delete height_property_;
    // delete line_width_property_;
    // delete show_border_property_;
    // delete auto_color_change_property_;
    // delete max_color_property_;
    // delete update_interval_property_;
    // delete show_caption_property_;
    // delete text_size_property_;
    // delete min_value_property_;
    // delete max_value_property_;
    // delete auto_color_change_property_;
  }

  void Plotter2DDisplay::initializeBuffer()
  {
    buffer_.resize(buffer_length_);
    if (min_value_ == 0.0 && max_value_ == 0.0) {
      min_value_ = -1.0;
      max_value_ = 1.0;
    }
    for (size_t i = 0; i < buffer_length_; i++) {
      buffer_[i] = 0.0;
    }
  }

  void Plotter2DDisplay::onInitialize()
  {
    static int count = 0;
    rviz::UniformStringStream ss;
    ss << "Plotter2DDisplayObject" << count++;
    overlay_.reset(new OverlayObject(ss.str()));
    updateBufferSize();
    onEnable();
    updateShowValue();
    updateWidth();
    updateHeight();
    updateLeft();
    updateTop();
    updateFGColor();
    updateBGColor();
    updateFGAlpha();
    updateBGAlpha();
    updateLineWidth();
    updateUpdateInterval();
    updateShowBorder();
    updateAutoColorChange();
    updateMaxColor();
    updateShowCaption();
    updateTextSize();
    updateAutoScale();
    updateMinValue();
    updateMaxValue();
    overlay_->updateTextureSize(width_property_->getInt(),
                                height_property_->getInt() + caption_offset_);
  }

  void Plotter2DDisplay::drawPlot()
  {
    QColor fg_color(fg_color_);
    QColor bg_color(bg_color_);
    
    fg_color.setAlpha(fg_alpha_);
    bg_color.setAlpha(bg_alpha_);

    if (auto_color_change_) {
      double r
        = std::min(std::max((buffer_[buffer_.size() - 1] - min_value_) / (max_value_ - min_value_),
                            0.0), 1.0);
      if (r > 0.3) {
        double r2 = (r - 0.3) / 0.7;
        fg_color.setRed((max_color_.red() - fg_color_.red()) * r2
                        + fg_color_.red());
        fg_color.setGreen((max_color_.green() - fg_color_.green()) * r2
                          + fg_color_.green());
        fg_color.setBlue((max_color_.blue() - fg_color_.blue()) * r2
                         + fg_color_.blue());
      }
    }
    
    {
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QImage Hud = buffer.getQImage(*overlay_);
      // initilize by the background color
      for (int i = 0; i < overlay_->getTextureWidth(); i++) {
        for (int j = 0; j < overlay_->getTextureHeight(); j++) {
          Hud.setPixel(i, j, bg_color.rgba());
        }
      }
      // paste in HUD speedometer. I resize the image and offset it by 8 pixels from
      // the bottom left edge of the render window
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);
      painter.setPen(QPen(fg_color, line_width_, Qt::SolidLine));
      
      uint16_t w = overlay_->getTextureWidth();
      uint16_t h = overlay_->getTextureHeight() - caption_offset_;

      double margined_max_value = max_value_ + (max_value_ - min_value_) / 2;
      double margined_min_value = min_value_ - (max_value_ - min_value_) / 2;
      
      for (size_t i = 1; i < buffer_length_; i++) {
        double v_prev = (margined_max_value - buffer_[i - 1]) / (margined_max_value - margined_min_value);
        double v = (margined_max_value - buffer_[i]) / (margined_max_value - margined_min_value);
        double u_prev = (i - 1) / (float)buffer_length_;
        double u = i / (float)buffer_length_;

        // chop within 0 ~ 1
        v_prev = std::max(std::min(v_prev, 1.0), 0.0);
        u_prev = std::max(std::min(u_prev, 1.0), 0.0);
        v = std::max(std::min(v, 1.0), 0.0);
        u = std::max(std::min(u, 1.0), 0.0);
        
        uint16_t x_prev = (int)(u_prev * w);
        uint16_t x = (int)(u * w);
        uint16_t y_prev = (int)(v_prev * h);
        uint16_t y = (int)(v * h);
        painter.drawLine(x_prev, y_prev, x, y);
      }
      // draw border
      if (show_border_) {
        painter.drawLine(0, 0, 0, h);
        painter.drawLine(0, h, w, h);
        painter.drawLine(w, h, w, 0);
        painter.drawLine(w, 0, 0, 0);
      }
      // draw caption
      if (show_caption_) {
        QFont font = painter.font();
        font.setPointSize(text_size_);
        font.setBold(true);
        painter.setFont(font);
        painter.drawText(0, h, w, caption_offset_,
                         Qt::AlignCenter | Qt::AlignVCenter,
                         getName());
      }
      if (show_value_) {
        QFont font = painter.font();
        font.setPointSize(w / 4);
        font.setBold(true);
        painter.setFont(font);
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << buffer_[buffer_.size() - 1];
        painter.drawText(0, 0, w, h,
                         Qt::AlignCenter | Qt::AlignVCenter,
                         ss.str().c_str());
      }
      
      // done
      painter.end();
    }
  }
  
  void Plotter2DDisplay::processMessage(const std_msgs::Float32::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!isEnabled()) {
      return;
    }
    // add the message to the buffer
    double min_value = buffer_[0];
    double max_value = buffer_[0];
    for (size_t i = 0; i < buffer_length_ - 1; i++) {
      buffer_[i] = buffer_[i + 1];
      if (min_value > buffer_[i]) {
        min_value = buffer_[i];
      }
      if (max_value < buffer_[i]) {
        max_value = buffer_[i];
      }
    }
    buffer_[buffer_length_ - 1] = msg->data;
    if (min_value > msg->data) {
      min_value = msg->data;
    }
    if (max_value < msg->data) {
      max_value = msg->data;
    }
    if (auto_scale_) {
      min_value_ = min_value;
      max_value_ = max_value;
      if (min_value_ == max_value_) {
        min_value_ = min_value_ - 0.5;
        max_value_ = max_value_ + 0.5;
      }
    }
    if (!overlay_->isVisible()) {
      return;
    }
    
    draw_required_ = true;
  }

  void Plotter2DDisplay::update(float wall_dt, float ros_dt)
  {
    if (draw_required_) {
      if (wall_dt + last_time_ > update_interval_) {
        overlay_->updateTextureSize(texture_width_,
                                    texture_height_ + caption_offset_);
        overlay_->setPosition(left_, top_);
        overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
        last_time_ = 0;
        drawPlot();
        draw_required_ = false;
      }
      else {
        last_time_ = last_time_ + wall_dt;
      }
    }
  }
  
  void Plotter2DDisplay::subscribe()
  {
    initializeBuffer();
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      ros::NodeHandle n;
      sub_ = n.subscribe(topic_name, 1, &Plotter2DDisplay::processMessage, this);
    }
  }

  void Plotter2DDisplay::unsubscribe()
  {
    sub_.shutdown();
  }

  void Plotter2DDisplay::onEnable()
  {
    last_time_ = 0;
    draw_required_ = false;
    subscribe();
    overlay_->show();
  }

  void Plotter2DDisplay::onDisable()
  {
    unsubscribe();
    overlay_->hide();
  }

  void Plotter2DDisplay::updateWidth()
  {
    boost::mutex::scoped_lock lock(mutex_);
    texture_width_ = width_property_->getInt();
  }
  
  void Plotter2DDisplay::updateHeight()
  {
    boost::mutex::scoped_lock lock(mutex_);
    texture_height_ = height_property_->getInt();
  }
  
  void Plotter2DDisplay::updateTop()
  {
    top_ = top_property_->getInt();
  }
  
  void Plotter2DDisplay::updateLeft()
  {
    left_ = left_property_->getInt();
  }
  
  void Plotter2DDisplay::updateBGColor()
  {
    bg_color_ = bg_color_property_->getColor();
  }

  void Plotter2DDisplay::updateFGColor()
  {
    fg_color_ = fg_color_property_->getColor();
  }

  void Plotter2DDisplay::updateFGAlpha()
  {
    fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
  }

  void Plotter2DDisplay::updateBGAlpha()
  {
    bg_alpha_ = bg_alpha_property_->getFloat() * 255.0;
  }
  
  void Plotter2DDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }

  void Plotter2DDisplay::updateShowValue()
  {
    show_value_ = show_value_property_->getBool();
  }
  
  void Plotter2DDisplay::updateShowBorder()
  {
    show_border_ = show_border_property_->getBool();
  }
  
  void Plotter2DDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getInt();
  }
  
  void Plotter2DDisplay::updateBufferSize()
  {
    buffer_length_ = buffer_length_property_->getInt();
    initializeBuffer();
  }

  void Plotter2DDisplay::updateAutoColorChange()
  {
    auto_color_change_ = auto_color_change_property_->getBool();
    if (auto_color_change_) {
      max_color_property_->show();
    }
    else {
      max_color_property_->hide();
    }
  }

  void Plotter2DDisplay::updateMaxColor()
  {
    max_color_ = max_color_property_->getColor();
  }
  
  void Plotter2DDisplay::updateUpdateInterval()
  {
    update_interval_ = update_interval_property_->getFloat();
  }

  void Plotter2DDisplay::updateTextSize()
  {
    text_size_ = text_size_property_->getInt();
    QFont font;
    font.setPointSize(text_size_);
    caption_offset_ = QFontMetrics(font).height();
  }
  
  void Plotter2DDisplay::updateShowCaption()
  {
    show_caption_  = show_caption_property_->getBool();
    if (show_caption_) {
      text_size_property_->show();
    }
    else {
      text_size_property_->hide();
    }
  }

  void Plotter2DDisplay::updateMinValue()
  {
    if (!auto_scale_) {
      min_value_ = min_value_property_->getFloat();
    }
  }

  void Plotter2DDisplay::updateMaxValue()
  {
    if (!auto_scale_) {
      max_value_ = max_value_property_->getFloat();
    }
  }

  void Plotter2DDisplay::updateAutoScale()
  {
    auto_scale_ = auto_scale_property_->getBool();
    if (auto_scale_) {
      min_value_property_->hide();
      max_value_property_->hide();
    }
    else {
      min_value_property_->show();
      max_value_property_->show();
    }
    updateMinValue();
    updateMaxValue();
  }

  bool Plotter2DDisplay::isInRegion(int x, int y)
  {
    return (top_ < y && top_ + texture_height_ > y &&
            left_ < x && left_ + texture_width_ > x);
  }

  void Plotter2DDisplay::movePosition(int x, int y)
  {
    top_ = y;
    left_ = x;
  }

  void Plotter2DDisplay::setPosition(int x, int y)
  {
    top_property_->setValue(y);
    left_property_->setValue(x);
  }

  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::Plotter2DDisplay, rviz::Display )
