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

#include "diagnostics_display.h"
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

namespace jsk_rviz_plugins
{

  DiagnosticsDisplay::DiagnosticsDisplay()
    : rviz::Display(), msg_(0)
  {
    ros_topic_property_
      = new rviz::RosTopicProperty(
        "Topic", "/diagnostics_agg",
        ros::message_traits::datatype<diagnostic_msgs::DiagnosticArray>(),
        "diagnostic_msgs::DiagnosticArray topic to subscribe to.",
        this, SLOT( updateRosTopic() ));
    frame_id_property_ = new rviz::TfFrameProperty(
      "frame_id", rviz::TfFrameProperty::FIXED_FRAME_STRING,
      "the parent frame_id to visualize diagnostics",
      this, 0, true);
    diagnostics_namespace_property_ = new rviz::EditableEnumProperty(
      "diagnostics namespace", "/",
      "diagnostics namespace to visualize diagnostics",
      this, SLOT(updateDiagnosticsNamespace()));
    radius_property_ = new rviz::FloatProperty(
      "radius", 1.0,
      "radius of diagnostics circle",
      this, SLOT(updateRadius()));
    line_width_property_ = new rviz::FloatProperty(
      "line width", 0.03,
      "line width",
      this, SLOT(updateLineWidth()));
    axis_property_ = new rviz::EnumProperty(
      "axis", "x",
      "axis",
      this, SLOT(updateAxis()));
    axis_property_->addOption("x", 0);
    axis_property_->addOption("y", 1);
    axis_property_->addOption("z", 2);
    font_size_property_ = new rviz::FloatProperty(
      "font size", 0.05,
      "font size",
      this, SLOT(updateFontSize()));
  }

  DiagnosticsDisplay::~DiagnosticsDisplay()
  {
    delete ros_topic_property_;
    delete frame_id_property_;
    delete diagnostics_namespace_property_;
    delete radius_property_;
    delete line_width_property_;
    delete axis_property_;
    delete line_;
    delete msg_;
    delete font_size_property_;
  }

  void DiagnosticsDisplay::update(float wall_dt, float ros_dt)
  {
    if (line_update_required_) {
      updateLine();
    }

    if (!isEnabled()) {
      return;
    }

    msg_->setCharacterHeight(font_size_);
    
    const float round_trip = 10.0;
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    std::string frame_id = frame_id_property_->getFrame().toStdString();
    if( !context_->getFrameManager()->getTransform( frame_id,
                                                    ros::Time(0.0),
                                                    position, orientation ))
    {
      ROS_WARN( "Error transforming from frame '%s' to frame '%s'",
                frame_id.c_str(), qPrintable( fixed_frame_ ));
      return;
    }
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    Ogre::Vector3 orbit_position;
    orbit_theta_ = ros_dt / round_trip * M_PI * 2.0 + orbit_theta_;
    while (orbit_theta_ > M_PI * 2) {
      orbit_theta_ -= M_PI*2;
    }
    if (axis_ == 0) {           // x
      orbit_position.x = radius_ * cos(orbit_theta_);
      orbit_position.y = radius_ * sin(orbit_theta_);
      orbit_position.z = 0;
    }
    else if (axis_ == 1) {      // y
      orbit_position.y = radius_ * cos(orbit_theta_);
      orbit_position.z = radius_ * sin(orbit_theta_);
      orbit_position.x = 0;
    }
    else if (axis_ == 2) {      // z
      orbit_position.z = radius_ * cos(orbit_theta_);
      orbit_position.x = radius_ * sin(orbit_theta_);
      orbit_position.y = 0;
    }
    
    orbit_node_->setPosition(orbit_position);
    if (!isEnabled()) {
      return;
    }
    context_->queueRender();
  }
  
  void DiagnosticsDisplay::onInitialize()
  {
    static int counter = 0;
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    orbit_node_ = scene_node_->createChildSceneNode(); // ??
    line_ = new rviz::BillboardLine(context_->getSceneManager(), scene_node_);
    msg_ = new rviz::MovableText("not initialized", "Liberation Sans", 0.05);
    msg_->setTextAlignment(rviz::MovableText::H_CENTER,
                           rviz::MovableText::V_ABOVE);
    frame_id_property_->setFrameManager(context_->getFrameManager());
    orbit_node_->attachObject(msg_);
    msg_->setVisible(false);
    orbit_theta_ = M_PI * 2.0 / 6 * counter++;
    updateLineWidth();
    updateAxis();
    updateDiagnosticsNamespace();
    updateRadius();
    updateRosTopic();
    updateFontSize();
  }
  
  void DiagnosticsDisplay::processMessage
  (const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
  {
    if (!isEnabled()) {
      return;
    }

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
    
    const float alpha = 0.8;
    const Ogre::ColourValue OK(0.3568627450980392, 0.7529411764705882, 0.8705882352941177, alpha);
    const Ogre::ColourValue WARN(0.9411764705882353, 0.6784313725490196, 0.3058823529411765, alpha);
    const Ogre::ColourValue ERROR(0.8509803921568627, 0.3254901960784314, 0.30980392156862746, 0.5);
    const Ogre::ColourValue UNKNOWN(0.2, 0.2, 0.2, 0.5);
    Ogre::ColourValue color;
    std::string message;
    bool foundp = false;
    for (size_t i = 0; i < msg->status.size(); i++) {
      diagnostic_msgs::DiagnosticStatus status = msg->status[i];
      if (status.name == diagnostics_namespace_) {
        if (status.level == diagnostic_msgs::DiagnosticStatus::OK) {
          color = OK;
          message = status.message;
        }
        else if (status.level == diagnostic_msgs::DiagnosticStatus::WARN) {
          color = WARN;
          message = status.message;
        }
        else if (status.level == diagnostic_msgs::DiagnosticStatus::ERROR) {
          color = ERROR;
          message = status.message;
        }
        else {
          // unknwon
          color = UNKNOWN;
          message = "unknown";
        }
        foundp = true;
        break;
      }
    }

    if (!foundp) {
      color = UNKNOWN;
      message = "stall";
    }
    
    line_->setColor(color.r, color.g, color.b, color.a);
    Ogre::ColourValue font_color(color);
    font_color.a = 1.0;
    msg_->setColor(font_color);
    msg_->setCaption(diagnostics_namespace_ + "\n" + message);
    context_->queueRender();
  }

  void DiagnosticsDisplay::updateLine()
  {
    line_->clear();
    line_->setLineWidth(line_width_);
    line_->setNumLines(1);
    line_->setMaxPointsPerLine(1024);
    for (size_t i = 0; i < 128 + 1; i++) {
      Ogre::Vector3 step_position;
      const double theta = M_PI * 2.0 / 128 * i;
      if (axis_ == 0) {
        step_position.x = radius_ * cos(theta);
        step_position.y = radius_ * sin(theta);
        step_position.z = 0.0;
      }
      else if (axis_ == 1) {
        step_position.y = radius_ * cos(theta);
        step_position.z = radius_ * sin(theta);
        step_position.x = 0;
      }
      else if (axis_ == 2) {
        step_position.z = radius_ * cos(theta);
        step_position.x = radius_ * sin(theta);
        step_position.y = 0;
      }
      line_->addPoint(step_position);
    }
    line_update_required_ = false;
  }
  
  void DiagnosticsDisplay::onEnable()
  {
    line_update_required_ = true;
    msg_->setVisible(true);
  }

  void DiagnosticsDisplay::onDisable()
  {
    unsubscribe();
    line_->clear();
    msg_->setVisible(false);
  }
  
  void DiagnosticsDisplay::unsubscribe()
  {
    sub_.shutdown();
  }
  
  void DiagnosticsDisplay::subscribe()
  {
    ros::NodeHandle n;
    sub_ = n.subscribe(ros_topic_property_->getTopicStd(),
                       1,
                       &DiagnosticsDisplay::processMessage,
                       this);
  }
  
  void DiagnosticsDisplay::updateRosTopic()
  {
    unsubscribe();
    subscribe();
  }

  void DiagnosticsDisplay::updateDiagnosticsNamespace()
  {
    diagnostics_namespace_ = diagnostics_namespace_property_->getStdString();
  }
  
  void DiagnosticsDisplay::updateRadius()
  {
    radius_ = radius_property_->getFloat();
    line_update_required_ = true;
  }

  void DiagnosticsDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getFloat();
    line_update_required_ = true;
  }


  void DiagnosticsDisplay::updateAxis()
  {
    axis_ = axis_property_->getOptionInt();
    line_update_required_ = true;
  }

  void DiagnosticsDisplay::fillNamespaceList()
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

  void DiagnosticsDisplay::updateFontSize()
  {
    font_size_ = font_size_property_->getFloat();
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::DiagnosticsDisplay, rviz::Display )
