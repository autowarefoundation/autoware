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
#ifndef JSK_RVIZ_PLUGIN_OVERLAY_DIAGNOSTIC_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_DIAGNOSTIC_DISPLAY_H_

#include <rviz/display.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>

#include <QPainter>

#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>

#include <diagnostic_msgs/DiagnosticArray.h>

#include "overlay_utils.h"

namespace jsk_rviz_plugins
{
  class OverlayDiagnosticDisplay: public rviz::Display
  {
    Q_OBJECT
  public:
    OverlayDiagnosticDisplay();
    virtual ~OverlayDiagnosticDisplay();
    
  protected:
    virtual bool isStalled();
    virtual void processMessage(
      const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
    virtual void update(float wall_dt, float ros_dt);
    virtual void onEnable();
    virtual void onDisable();
    virtual void onInitialize();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void redraw();
    virtual void fillNamespaceList();
    virtual QColor foregroundColor();
    virtual QColor blendColor(QColor a, QColor b, double a_rate);
    virtual void drawText(QPainter& painter, QColor fg_color,
                          const std::string& text);
    virtual double drawAnimatingText(QPainter& painter,
                                     QColor fg_color,
                                     const double height,
                                     const double font_size,
                                     const std::string text);
    virtual std::string statusText();
    boost::mutex mutex_;
    OverlayObject::Ptr overlay_;
    
    diagnostic_msgs::DiagnosticStatus::Ptr latest_status_;
    ros::WallTime latest_message_time_;
    int size_;
    std::string diagnostics_namespace_;
    std::set<std::string> namespaces_;
    double alpha_;
    int top_, left_;
    double t_;
    double stall_duration_;
    rviz::RosTopicProperty* ros_topic_property_;
    rviz::EditableEnumProperty* diagnostics_namespace_property_;
    rviz::IntProperty* top_property_;
    rviz::IntProperty* left_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::IntProperty* size_property_;
    rviz::FloatProperty* stall_duration_property_;
    
    ros::Subscriber sub_;
  protected Q_SLOTS:
    virtual void updateRosTopic();
    virtual void updateDiagnosticsNamespace();
    virtual void updateSize();
    virtual void updateAlpha();
    virtual void updateTop();
    virtual void updateLeft();
    virtual void updateStallDuration();
  private:
    
  };
}

#endif
