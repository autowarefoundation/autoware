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

#include "overlay_diagnostic_display.h"
#include <OGRE/OgreOverlayManager.h>
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
  const double overlay_diagnostic_animation_duration = 5.0;
  OverlayDiagnosticDisplay::OverlayDiagnosticDisplay()
    : Display()
  {
    ros_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "/diagnostics_agg",
      ros::message_traits::datatype<diagnostic_msgs::DiagnosticArray>(),
      "diagnostic_msgs::DiagnosticArray topic to subscribe to.",
      this, SLOT( updateRosTopic() ));
    diagnostics_namespace_property_ = new rviz::EditableEnumProperty(
      "diagnostics namespace", "/",
      "diagnostics namespace to visualize diagnostics",
      this, SLOT(updateDiagnosticsNamespace()));
    top_property_ = new rviz::IntProperty(
      "top", 128,
      "top positoin",
      this, SLOT(updateTop()));
    left_property_ = new rviz::IntProperty(
      "left", 128,
      "left positoin",
      this, SLOT(updateLeft()));
    size_property_ = new rviz::IntProperty(
      "size", 128,
      "size of the widget",
      this, SLOT(updateSize()));
    size_property_->setMin(1);
    alpha_property_ = new rviz::FloatProperty(
      "alpha", 0.8,
      "alpha value",
      this, SLOT(updateAlpha()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);
    stall_duration_property_ = new rviz::FloatProperty(
      "stall duration", 5.0,
      "seconds to be regarded as stalled",
      this, SLOT(updateStallDuration())
      );
    stall_duration_property_->setMin(0.0);
  }

  OverlayDiagnosticDisplay::~OverlayDiagnosticDisplay()
  {
    if (overlay_) {
      overlay_->hide();
    }
    // panel_material_->unload();
    // Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
    delete ros_topic_property_;
    delete diagnostics_namespace_property_;
    delete top_property_;
    delete left_property_;
    delete alpha_property_;
    delete size_property_;
  }

  void OverlayDiagnosticDisplay::processMessage(
    const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex_);
    //boost::make_shared<diagnostic_msgs::DiagnosticStatus>
    // update namespaces_ if needed
    std::set<std::string> new_namespaces;
    for (size_t i = 0; i < msg->status.size(); i++) {
      new_namespaces.insert(msg->status[i].name);
    }
    
    std::set<std::string> difference_namespaces;
    std::set_difference(namespaces_.begin(), namespaces_.end(),
                        new_namespaces.begin(), new_namespaces.end(),
                        std::inserter(difference_namespaces,
                                      difference_namespaces.end()));
    if (difference_namespaces.size() != 0) {
      namespaces_ = new_namespaces;
      fillNamespaceList();
    }
    else {
      difference_namespaces.clear();
      std::set_difference(new_namespaces.begin(), new_namespaces.end(),
                          namespaces_.begin(), namespaces_.end(),
                          std::inserter(difference_namespaces,
                                        difference_namespaces.end()));
      if (difference_namespaces.size() != 0) {
        namespaces_ = new_namespaces;
        fillNamespaceList();
      }
    }
    
    if (diagnostics_namespace_.length() == 0) {
      return;
    }

    for (size_t i = 0; i < msg->status.size(); i++) {
      diagnostic_msgs::DiagnosticStatus status = msg->status[i];
      if (status.name == diagnostics_namespace_) {
        latest_status_
          = boost::make_shared<diagnostic_msgs::DiagnosticStatus>(status);
        latest_message_time_ = ros::WallTime::now();
        break;
      }
    }
  }

  void OverlayDiagnosticDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock(mutex_);
    if (!isEnabled()) {
      return;
    }
    if (!overlay_) {
      static int count = 0;
      rviz::UniformStringStream ss;
      ss << "OverlayDiagnosticDisplayObject" << count++;
      overlay_.reset(new OverlayObject(ss.str()));
      overlay_->show();
    }
    t_ += wall_dt;
    
    overlay_->updateTextureSize(size_, size_);
    redraw();
    overlay_->setDimensions(overlay_->getTextureWidth(),
                            overlay_->getTextureHeight());
    overlay_->setPosition(left_, top_);
    t_ = fmod(t_, overlay_diagnostic_animation_duration);
  }

  void OverlayDiagnosticDisplay::onEnable()
  {
    t_ = 0.0;
    if (overlay_) {
      overlay_->show();
    }
    subscribe();
  }

  void OverlayDiagnosticDisplay::onDisable()
  {
    ROS_INFO("onDisable");
    if (overlay_) {
      overlay_->hide();
    }
    unsubscribe();
  }

  void OverlayDiagnosticDisplay::onInitialize()
  {
    
    ROS_DEBUG("onInitialize");


    updateDiagnosticsNamespace();
    updateSize();
    updateAlpha();
    updateLeft();
    updateTop();
    updateStallDuration();
    updateRosTopic();
  }
  
  void OverlayDiagnosticDisplay::unsubscribe()
  {
    sub_.shutdown();
  }
  
  void OverlayDiagnosticDisplay::subscribe()
  {
    ros::NodeHandle n;
    sub_ = n.subscribe(ros_topic_property_->getTopicStd(),
                       1,
                       &OverlayDiagnosticDisplay::processMessage,
                       this);
  }

  bool OverlayDiagnosticDisplay::isStalled()
  {
    if (latest_status_) {
      ros::WallDuration message_duration
        = ros::WallTime::now() - latest_message_time_;
      if (message_duration.toSec() < stall_duration_) {
        return false;
      }
      else {
        return true;
      }
    }
    else {
      return true;
    }
  }
  
  std::string OverlayDiagnosticDisplay::statusText()
  {
    if (latest_status_) {
      if (!isStalled()) {
        if (latest_status_->level == diagnostic_msgs::DiagnosticStatus::OK) {
          return "OK";
        }
        else if (latest_status_->level == diagnostic_msgs::DiagnosticStatus::WARN) {
          return "WARN";
        }
        else if (latest_status_->level == diagnostic_msgs::DiagnosticStatus::ERROR) {
          return "ERROR";
        }
        else {
          return "UNKNOWN";
        }
      }
      else {
        return "UNKNOWN";
      }
    }
    else {
      return "UNKNOWN";
    }
  }
  
  
  QColor OverlayDiagnosticDisplay::foregroundColor()
  {
    QColor ok_color(25, 255, 240, alpha_ * 255.0);
    QColor warn_color(240, 173, 78, alpha_ * 255.0);
    QColor error_color(217, 83, 79, alpha_ * 255.0);
    QColor stall_color(151, 151, 151, alpha_ * 255.0);
    //QColor fg_color = stall_color;
    
    if (latest_status_) {
      if (!isStalled()) {
        if (latest_status_->level == diagnostic_msgs::DiagnosticStatus::OK) {
          return ok_color;
        }
        else if (latest_status_->level
                 == diagnostic_msgs::DiagnosticStatus::WARN) {
          return warn_color;
        }
        else if (latest_status_->level
                 == diagnostic_msgs::DiagnosticStatus::ERROR) {
          return error_color;
        }
        else {
          return stall_color;
        }
      }
      else {
        return stall_color;
      }
    }
    else {
      return stall_color;
    }
  }

  QColor OverlayDiagnosticDisplay::blendColor(QColor a, QColor b, double a_rate) {
    QColor ret (a.red() * a_rate + b.red() * (1 - a_rate),
                a.green() * a_rate + b.green() * (1 - a_rate),
                a.blue() * a_rate + b.blue() * (1 - a_rate),
                a.alpha() * a_rate + b.alpha() * (1 - a_rate));
    return ret;
  }

  double OverlayDiagnosticDisplay::drawAnimatingText(QPainter& painter,
                                                     QColor fg_color,
                                                     const double height,
                                                     const double font_size,
                                                     const std::string text)
  {
    const double r = size_ / 128.0;
    QFont font("Arial", font_size * r, font_size * r, false);
    QPen pen;
    QPainterPath path;
    pen.setWidth(1);
    pen.setColor(blendColor(fg_color, Qt::white, 0.5));
    painter.setFont(font);
    painter.setPen(pen);
    painter.setBrush(fg_color);
    QFontMetrics metrics(font);
    const int text_width = metrics.width(text.c_str());
    const int text_height = metrics.height();
    if (overlay_->getTextureWidth() > text_width) {
      path.addText((overlay_->getTextureWidth() - text_width) / 2.0,
                   height,
                   font, text.c_str());
    }
    else {
      double left = - fmod(t_, overlay_diagnostic_animation_duration) /
        overlay_diagnostic_animation_duration * text_width * 2 + text_width;
      path.addText(left, height, font, text.c_str());
    }
    painter.drawPath(path);
    return text_height;
  }
  
  void OverlayDiagnosticDisplay::drawText(QPainter& painter, QColor fg_color,
                                          const std::string& text)
  {
    double status_size = drawAnimatingText(painter, fg_color,
                                           overlay_->getTextureHeight() / 3.0,
                                           20, text);
    double namespace_size = drawAnimatingText(painter, fg_color,
                                              overlay_->getTextureHeight() / 3.0 + status_size,
                                              10, diagnostics_namespace_);
    std::string message;
    if (latest_status_) {
      if (!isStalled()) {
        message = latest_status_->message;
      }
      else {
        message = "stalled";
      }
    }
    else {
      message = "stalled";
    }
    drawAnimatingText(painter, fg_color,
                      overlay_->getTextureHeight() / 3.0
                      + status_size + namespace_size,
                      10, message);
  }
  
  void OverlayDiagnosticDisplay::redraw()
  {
    ScopedPixelBuffer buffer = overlay_->getBuffer();
    QColor fg_color = foregroundColor();
    QColor transparent(0, 0, 0, 0.0);
    
    QImage Hud = buffer.getQImage(*overlay_, transparent);
    // draw outer circle
    // line-width - margin - inner-line-width < size
    QPainter painter( &Hud );
    const int line_width = 10;
    const int margin = 10;
    const int inner_line_width = 20;
    
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(fg_color, line_width, Qt::SolidLine));
    painter.drawEllipse(line_width / 2, line_width / 2,
                        overlay_->getTextureWidth() - line_width,
                        overlay_->getTextureHeight() - line_width);
    
    painter.setPen(QPen(fg_color, inner_line_width, Qt::SolidLine));    
    const double start_angle = fmod(t_, overlay_diagnostic_animation_duration) /
      overlay_diagnostic_animation_duration * 360;
    const double draw_angle = 250;
    const double inner_circle_start
      = line_width + margin + inner_line_width / 2.0;
    drawText(painter, fg_color, statusText());
  }

  void OverlayDiagnosticDisplay::fillNamespaceList()
  {
    //QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    diagnostics_namespace_property_->clearOptions();
    for (std::set<std::string>::iterator it = namespaces_.begin();
         it != namespaces_.end();
         it++) {
      diagnostics_namespace_property_->addOptionStd(*it);
    }
    diagnostics_namespace_property_->sortOptions();
  }
  
  void OverlayDiagnosticDisplay::updateRosTopic()
  {
    latest_status_.reset();
    unsubscribe();
    subscribe();
  }

  void OverlayDiagnosticDisplay::updateDiagnosticsNamespace()
  {
    latest_status_.reset();
    diagnostics_namespace_ = diagnostics_namespace_property_->getStdString();
  }
  
  void OverlayDiagnosticDisplay::updateSize()
  {
    size_ = size_property_->getInt();
  }

  void OverlayDiagnosticDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
  }

  void OverlayDiagnosticDisplay::updateTop()
  {
    top_ = top_property_->getInt();
  }
  
  void OverlayDiagnosticDisplay::updateLeft()
  {
    left_ = left_property_->getInt();
  }
  
  void OverlayDiagnosticDisplay::updateStallDuration()
  {
    stall_duration_ = stall_duration_property_->getFloat();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::OverlayDiagnosticDisplay, rviz::Display)
