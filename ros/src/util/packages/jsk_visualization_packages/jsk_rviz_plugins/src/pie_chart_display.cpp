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

#include "pie_chart_display.h"

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/display_context.h>
#include <QPainter>

namespace jsk_rviz_plugins
{

  PieChartDisplay::PieChartDisplay()
    : rviz::Display(), update_required_(false), first_time_(true), data_(0.0)
  {
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      ros::message_traits::datatype<std_msgs::Float32>(),
      "std_msgs::Float32 topic to subscribe to.",
      this, SLOT( updateTopic() ));
    size_property_ = new rviz::IntProperty("size", 128,
                                           "size of the plotter window",
                                           this, SLOT(updateSize()));
    left_property_ = new rviz::IntProperty("left", 128,
                                           "left of the plotter window",
                                           this, SLOT(updateLeft()));
    top_property_ = new rviz::IntProperty("top", 128,
                                          "top of the plotter window",
                                          this, SLOT(updateTop()));
    fg_color_property_ = new rviz::ColorProperty("foreground color",
                                                 QColor(25, 255, 240),
                                                 "color to draw line",
                                                 this, SLOT(updateFGColor()));
    fg_alpha_property_
      = new rviz::FloatProperty("foreground alpha", 0.7,
                                "alpha belnding value for foreground",
                                this, SLOT(updateFGAlpha()));
    fg_alpha2_property_
      = new rviz::FloatProperty("foreground alpha 2", 0.4,
                                "alpha belnding value for foreground for indicator",
                                this, SLOT(updateFGAlpha2()));
    bg_color_property_ = new rviz::ColorProperty("background color",
                                                 QColor(0, 0, 0),
                                                 "background color",
                                                 this, SLOT(updateBGColor()));
    bg_alpha_property_
      = new rviz::FloatProperty("backround alpha", 0.0,
                                "alpha belnding value for background",
                                this, SLOT(updateBGAlpha()));
    text_size_property_
      = new rviz::IntProperty("text size", 14,
                              "text size",
                              this, SLOT(updateTextSize()));
    show_caption_property_
      = new rviz::BoolProperty("show caption", true,
                                "show caption",
                                this, SLOT(updateShowCaption()));
    max_value_property_
      = new rviz::FloatProperty("max value", 1.0,
                                "max value of pie chart",
                                this, SLOT(updateMaxValue()));
    min_value_property_
      = new rviz::FloatProperty("min value", 0.0,
                                "min value of pie chart",
                                this, SLOT(updateMinValue()));
    auto_color_change_property_
      = new rviz::BoolProperty("auto color change",
                               false,
                               "change the color automatically",
                               this, SLOT(updateAutoColorChange()));
    max_color_property_
      = new rviz::ColorProperty("max color",
                                QColor(255, 0, 0),
                                "only used if auto color change is set to True.",
                                this, SLOT(updateMaxColor()));

  }

  PieChartDisplay::~PieChartDisplay()
  {
    if (overlay_->isVisible()) {
      overlay_->hide();
    }
    delete update_topic_property_;
    delete fg_color_property_;
    delete bg_color_property_;
    delete fg_alpha_property_;
    delete fg_alpha2_property_;
    delete bg_alpha_property_;
    delete top_property_;
    delete left_property_;
    delete size_property_;
    delete min_value_property_;
    delete max_value_property_;
    delete text_size_property_;
    delete show_caption_property_;
  }

  void PieChartDisplay::onInitialize()
  {
    static int count = 0;
    rviz::UniformStringStream ss;
    ss << "PieChartDisplayObject" << count++;
    overlay_.reset(new OverlayObject(ss.str()));
    onEnable();
    updateSize();
    updateLeft();
    updateTop();
    updateFGColor();
    updateBGColor();
    updateFGAlpha();
    updateFGAlpha2();
    updateBGAlpha();
    updateMinValue();
    updateMaxValue();
    updateTextSize();
    updateShowCaption();
    updateAutoColorChange();
    updateMaxColor();
    overlay_->updateTextureSize(texture_size_, texture_size_ + caption_offset_);
    overlay_->hide();
  }

  void PieChartDisplay::update(float wall_dt, float ros_dt)
  {
    if (update_required_) {
      update_required_ = false;
      overlay_->updateTextureSize(texture_size_, texture_size_ + caption_offset_);
      drawPlot(data_);
      overlay_->setPosition(left_, top_);
      overlay_->setDimensions(overlay_->getTextureWidth(),
                              overlay_->getTextureHeight());
    }
  }
  
  void PieChartDisplay::processMessage(const std_msgs::Float32::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!overlay_->isVisible()) {
      return;
    }
    if (data_ != msg->data || first_time_) {
      first_time_ = false;
      data_ = msg->data;
      update_required_ = true;
    }
  }
  
  void PieChartDisplay::drawPlot(double val)
  {
    QColor fg_color(fg_color_);

    if (auto_color_change_) {
      double r
        = std::min(1.0, fabs((val - min_value_) / (max_value_ - min_value_)));
      if (r > 0.6) {
        double r2 = (r - 0.6) / 0.4;
        fg_color.setRed((max_color_.red() - fg_color_.red()) * r2
                        + fg_color_.red());
        fg_color.setGreen((max_color_.green() - fg_color_.green()) * r2
                          + fg_color_.green());
        fg_color.setBlue((max_color_.blue() - fg_color_.blue()) * r2
                         + fg_color_.blue());
      }
    }

    
    QColor fg_color2(fg_color);
    QColor bg_color(bg_color_);
    fg_color.setAlpha(fg_alpha_);
    fg_color2.setAlpha(fg_alpha2_);
    bg_color.setAlpha(bg_alpha_);
    int width = overlay_->getTextureWidth();
    int height = overlay_->getTextureHeight();
    {
      ScopedPixelBuffer buffer = overlay_->getBuffer();
      QImage Hud = buffer.getQImage(*overlay_, bg_color);
      QPainter painter( &Hud );
      painter.setRenderHint(QPainter::Antialiasing, true);

      const int outer_line_width = 5;
      const int value_line_width = 10;
      const int value_indicator_line_width = 2;
      const int value_padding = 5;

      const int value_aabb_offset
        = outer_line_width + value_padding + value_line_width / 2;
      
      painter.setPen(QPen(fg_color, outer_line_width, Qt::SolidLine));

      painter.drawEllipse(outer_line_width / 2, outer_line_width / 2,
                          width - outer_line_width ,
                          height - outer_line_width - caption_offset_);

      painter.setPen(QPen(fg_color2, value_indicator_line_width, Qt::SolidLine));
      painter.drawEllipse(value_aabb_offset, value_aabb_offset,
                          width - value_aabb_offset * 2,
                          height - value_aabb_offset * 2 - caption_offset_);

      const double ratio = (val - min_value_) / (max_value_ - min_value_);
      const double ratio_angle = ratio * 360.0;
      const double start_angle_offset = -90;
      painter.setPen(QPen(fg_color, value_line_width, Qt::SolidLine));
      painter.drawArc(QRectF(value_aabb_offset, value_aabb_offset,
                             width - value_aabb_offset * 2,
                             height - value_aabb_offset * 2 - caption_offset_),
                      start_angle_offset * 16 ,
                      ratio_angle * 16);
      QFont font = painter.font();
      font.setPointSize(text_size_);
      font.setBold(true);
      painter.setFont(font);
      painter.setPen(QPen(fg_color, value_line_width, Qt::SolidLine));
      std::ostringstream s;
      s << std::fixed << std::setprecision(2) << val;
      painter.drawText(0, 0, width, height - caption_offset_,
                       Qt::AlignCenter | Qt::AlignVCenter,
                       s.str().c_str());

      // caption
      if (show_caption_) {
        painter.drawText(0, height - caption_offset_, width, caption_offset_,
                         Qt::AlignCenter | Qt::AlignVCenter,
                         getName());
      }
      
      // done
      painter.end();
      // Unlock the pixel buffer
    }
  }

  
  void PieChartDisplay::subscribe()
  {
    std::string topic_name = update_topic_property_->getTopicStd();
    if (topic_name.length() > 0 && topic_name != "/") {
      ros::NodeHandle n;
      sub_ = n.subscribe(topic_name, 1, &PieChartDisplay::processMessage, this);
    }
  }

  
  void PieChartDisplay::unsubscribe()
  {
    sub_.shutdown();
  }

  void PieChartDisplay::onEnable()
  {
    subscribe();
    overlay_->show();
    first_time_ = true;
  }

  void PieChartDisplay::onDisable()
  {
    unsubscribe();
    overlay_->hide();
  }

  void PieChartDisplay::updateSize()
  {
    boost::mutex::scoped_lock lock(mutex_);
    texture_size_ = size_property_->getInt();
  }
  
  void PieChartDisplay::updateTop()
  {
    top_ = top_property_->getInt();
  }
  
  void PieChartDisplay::updateLeft()
  {
    left_ = left_property_->getInt();
  }
  
  void PieChartDisplay::updateBGColor()
  {
    bg_color_ = bg_color_property_->getColor();
  }

  void PieChartDisplay::updateFGColor()
  {
    fg_color_ = fg_color_property_->getColor();
  }

  void PieChartDisplay::updateFGAlpha()
  {
    fg_alpha_ = fg_alpha_property_->getFloat() * 255.0;
  }

  void PieChartDisplay::updateFGAlpha2()
  {
    fg_alpha2_ = fg_alpha2_property_->getFloat() * 255.0;
  }

  
  void PieChartDisplay::updateBGAlpha()
  {
    bg_alpha_ = bg_alpha_property_->getFloat() * 255.0;
  }

  void PieChartDisplay::updateMinValue()
  {
    min_value_ = min_value_property_->getFloat();
  }

  void PieChartDisplay::updateMaxValue()
  {
    max_value_ = max_value_property_->getFloat();
  }
  
  void PieChartDisplay::updateTextSize()
  {
    boost::mutex::scoped_lock lock(mutex_);
    text_size_ = text_size_property_->getInt();
    QFont font;
    font.setPointSize(text_size_);
    caption_offset_ = QFontMetrics(font).height();
    
  }
  
  void PieChartDisplay::updateShowCaption()
  {
    show_caption_ = show_caption_property_->getBool();
  }

  
  void PieChartDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }

  void PieChartDisplay::updateAutoColorChange()
  {
    auto_color_change_ = auto_color_change_property_->getBool();
    if (auto_color_change_) {
      max_color_property_->show();
    }
    else {
      max_color_property_->hide();
    }
  }

  void PieChartDisplay::updateMaxColor()
  {
    max_color_ = max_color_property_->getColor();
  }

  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::PieChartDisplay, rviz::Display )
