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
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/uniform_string_stream.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreMaterialManager.h>
#include "target_visualizer_display.h"
#include <OGRE/OgreManualObject.h>

namespace jsk_rviz_plugins
{
  const float arrow_animation_duration = 1.0;
  const double minimum_font_size = 0.2;
  
  TargetVisualizerDisplay::TargetVisualizerDisplay():
    message_recieved_(false)
  {
    target_name_property_ = new rviz::StringProperty(
      "target name", "target",
      "name of the target",
      this, SLOT(updateTargetName())
      );
    radius_property_ = new rviz::FloatProperty(
      "radius", 1.0,
      "radius of the target mark",
      this, SLOT(updateRadius()));
    radius_property_->setMin(0.0);
    alpha_property_ = new rviz::FloatProperty(
      "alpha", 0.8,
      "0 is fully transparent, 1.0 is fully opaque.",
      this, SLOT(updateAlpha()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);
    color_property_ = new rviz::ColorProperty(
      "color", QColor(25, 255, 240),
      "color of the target",
      this, SLOT(updateColor()));
    shape_type_property_ = new rviz::EnumProperty(
      "type", "Simple Circle",
      "Shape to display the pose as",
      this, SLOT(updateShapeType()));
    shape_type_property_->addOption("Simple Circle", SimpleCircle);
    shape_type_property_->addOption("Decoreted Circle", GISCircle);
  }

  TargetVisualizerDisplay::~TargetVisualizerDisplay()
  {
    delete target_name_property_;
    delete alpha_property_;
    delete color_property_;
    delete radius_property_;
  }

  void TargetVisualizerDisplay::onEnable()
  {
    subscribe();
    visualizer_->setEnable(false); // keep false, it will be true
                                   // in side of processMessae callback.
  }
  
  void TargetVisualizerDisplay::processMessage(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    message_recieved_ = true;
    visualizer_->setEnable(isEnabled());
    if (!isEnabled()) {
      return;
    }
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if(!context_->getFrameManager()->transform(msg->header,
                                               msg->pose,
                                               position, orientation))
    {
      std::ostringstream oss;
      oss << "Error transforming pose";
      oss << " from frame '" << msg->header.frame_id << "'";
      oss << " to frame '" << qPrintable(fixed_frame_) << "'";
      ROS_ERROR_STREAM(oss.str());
      setStatus(rviz::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
      return;
    }
    visualizer_->setPosition(position);
  }

  
  void TargetVisualizerDisplay::update(float wall_dt, float ros_dt)
  {
    if (!message_recieved_) {
      return;
    }
    visualizer_->setOrientation(context_);
    visualizer_->update(wall_dt, ros_dt);
  }

  void TargetVisualizerDisplay::onInitialize()
  {
    visualizer_initialized_ = false;
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    
    updateRadius();
    updateShapeType();
    // updateTargetName();
    // updateColor();
    // updateAlpha();
  }

  void TargetVisualizerDisplay::reset()
  {
    MFDClass::reset();
    message_recieved_ = false;
    if (visualizer_) {
      visualizer_->setEnable(false);
    }
  }

  void TargetVisualizerDisplay::updateTargetName()
  {
    boost::mutex::scoped_lock lock(mutex_);
    target_name_ = target_name_property_->getStdString();
    if (visualizer_) {
      visualizer_->setText(target_name_);
    }
  }
  
  void TargetVisualizerDisplay::updateRadius()
  {
    boost::mutex::scoped_lock lock(mutex_);
    radius_ = radius_property_->getFloat();
    if (visualizer_) {
      visualizer_->setSize(radius_);
    }
  }

  void TargetVisualizerDisplay::updateAlpha()
  {
    boost::mutex::scoped_lock lock(mutex_);
    alpha_ = alpha_property_->getFloat();
    if (visualizer_) {
      visualizer_->setAlpha(alpha_);
    }
  }

  void TargetVisualizerDisplay::updateColor()
  {
    boost::mutex::scoped_lock lock(mutex_);
    color_ = color_property_->getColor();
    if (visualizer_) {
      visualizer_->setColor(color_);
    }
  }
  
  void TargetVisualizerDisplay::updateShapeType()
  {
    if (!visualizer_initialized_ ||
        current_type_ != shape_type_property_->getOptionInt()) {
      {
        boost::mutex::scoped_lock lock(mutex_);
        if (shape_type_property_->getOptionInt() == SimpleCircle) {
          current_type_ = SimpleCircle;
          // simple circle
          visualizer_.reset(new SimpleCircleFacingVisualizer(
                              scene_manager_,
                              scene_node_,
                              context_,
                              radius_));
        }
        else {
          current_type_ = GISCircle;
          // GIS
          GISCircleVisualizer* v = new GISCircleVisualizer(
            scene_manager_,
            scene_node_,
            radius_);
          v->setAnonymous(false);
          visualizer_.reset(v);
        }
        visualizer_initialized_ = true;
      }
      updateTargetName();
      updateColor();
      updateAlpha();
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::TargetVisualizerDisplay, rviz::Display )

