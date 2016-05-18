// -*- mode: c++ -*-
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

#include "footstep_display.h"
#include <rviz/validate_floats.h>
#include <jsk_topic_tools/color_utils.h>

namespace jsk_rviz_plugins
{
  FootstepDisplay::FootstepDisplay()
  {
    alpha_property_ =  new rviz::FloatProperty( "Alpha", 0.5,
                                                "0 is fully transparent, 1.0 is fully opaque.",
                                                this, SLOT( updateAlpha() ));
    show_name_property_ = new rviz::BoolProperty(
      "Show Name", true,
      "Show name of each footstep",
      this, SLOT(updateShowName()));
    use_group_coloring_property_ = new rviz::BoolProperty(
      "Use Group Coloring", false,
      "Use footstep_group field to colorize footsteps",
      this, SLOT(updateUseGroupColoring()));
    width_property_ =  new rviz::FloatProperty(
      "Width", 0.15,
      "width of the footstep, it's not used if the dimensions is specified in Footstep message.",
      this, SLOT( updateWidth() ));
    height_property_ =  new rviz::FloatProperty(
      "height", 0.01,
      "height of the footstep, it's not used if the dimensions is specified in Footstep message.",
      this, SLOT( updateHeight() ));

    depth_property_ =  new rviz::FloatProperty(
      "depth", 0.3,
      "depth of the footstep, it's not used if the dimensions is specified in Footstep message.",
      this, SLOT( updateDepth() ));
  }

  FootstepDisplay::~FootstepDisplay()
  {
    delete alpha_property_;
    delete width_property_;
    delete height_property_;
    delete depth_property_;
    delete show_name_property_;
    delete use_group_coloring_property_;
    delete line_;
    // remove all the nodes
    for (size_t i = 0; i < text_nodes_.size(); i++) {
      Ogre::SceneNode* node = text_nodes_[i];
      node->removeAndDestroyAllChildren();
      node->detachAllObjects();
      scene_manager_->destroySceneNode(node);
    }
  }

  void FootstepDisplay::updateWidth()
  {
    width_ = width_property_->getFloat();
  }

  void FootstepDisplay::updateHeight()
  {
    height_ = height_property_->getFloat();
  }

  void FootstepDisplay::updateDepth()
  {
    depth_ = depth_property_->getFloat();
  }
  
  void FootstepDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
  }

  void FootstepDisplay::updateShowName()
  {
    show_name_ = show_name_property_->getBool();
  }

  void FootstepDisplay::updateUseGroupColoring()
  {
    use_group_coloring_ = use_group_coloring_property_->getBool();
  }
  
  
  void FootstepDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
    line_->clear();
    allocateTexts(0);
  }

  bool FootstepDisplay::validateFloats( const jsk_footstep_msgs::FootstepArray& msg )
  {
    for (std::vector<jsk_footstep_msgs::Footstep>::const_iterator it = msg.footsteps.begin();
         it != msg.footsteps.end();
         ++it) {
      if (!rviz::validateFloats((*it).pose.position.x)
          || !rviz::validateFloats((*it).pose.position.y)
          || !rviz::validateFloats((*it).pose.position.z)
          || !rviz::validateFloats((*it).pose.orientation.x)
          || !rviz::validateFloats((*it).pose.orientation.y)
          || !rviz::validateFloats((*it).pose.orientation.z)
          || !rviz::validateFloats((*it).pose.orientation.w)
        ) {
        return false;
      }
    }
    return true;
  }

  void FootstepDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    line_ = new rviz::BillboardLine(context_->getSceneManager(), scene_node_);
    updateShowName();
    updateWidth();
    updateHeight();
    updateDepth();
    updateAlpha();
    updateUseGroupColoring();
  }

  void FootstepDisplay::allocateCubes(size_t num) {
    if (num > shapes_.size()) {
      // need to allocate
      for (size_t i = shapes_.size(); i < num; i++) {
        ShapePtr shape;
        shape.reset(new rviz::Shape(rviz::Shape::Cube, context_->getSceneManager(),
                                    scene_node_));
        shapes_.push_back(shape);
      }
    }
    else if (num < shapes_.size()) {
      // need to remove
      shapes_.resize(num);
    }
  }

  void FootstepDisplay::allocateTexts(size_t num) {
    if (num > texts_.size()) {
      // need to allocate
      for (size_t i = texts_.size(); i < num; i++) {
        // create nodes
        Ogre::SceneNode* node = scene_node_->createChildSceneNode();
        rviz::MovableText* text 
          = new rviz::MovableText("not initialized", "Arial", 0.05);
        text->setVisible(false);
        text->setTextAlignment(rviz::MovableText::H_CENTER,
                               rviz::MovableText::V_ABOVE);
        node->attachObject(text);
        texts_.push_back(text);
        text_nodes_.push_back(node);
      }
    }
    else if (num < texts_.size()) {
      for (int i = texts_.size() - 1; i >= (int)num; i--) {
        Ogre::SceneNode* node = text_nodes_[i];
        node->detachAllObjects();
        node->removeAndDestroyAllChildren();
        scene_manager_->destroySceneNode(node);
      }
      text_nodes_.resize(num);
      texts_.resize(num);
      
    }
  }

  double FootstepDisplay::minNotZero(double a, double b) {
    if (a == 0.0) {
      return b;
    }
    else if (b == 0.0) {
      return a;
    }
    else {
      return std::min(a, b);
    }
  }

  void FootstepDisplay::update(float wall_dt, float ros_dt)
  {
    for (size_t i = 0; i < shapes_.size(); i++) {
      ShapePtr shape = shapes_[i];
      texts_[i]->setVisible(show_name_); // TODO
      jsk_footstep_msgs::Footstep footstep = latest_footstep_->footsteps[i];
            // color
      if (use_group_coloring_) {
        std_msgs::ColorRGBA color
          = jsk_topic_tools::colorCategory20(footstep.footstep_group);
        shape->setColor(color.r, color.g, color.b, alpha_);
      }
      else {
        if (footstep.leg == jsk_footstep_msgs::Footstep::LEFT) {
          shape->setColor(0, 1, 0, alpha_);
        }
        else if (footstep.leg == jsk_footstep_msgs::Footstep::RIGHT) {
          shape->setColor(1, 0, 0, alpha_);
        }
        else {
          shape->setColor(1, 1, 1, alpha_);
        }
      }

    }
  }
  
  double FootstepDisplay::estimateTextSize(
    const jsk_footstep_msgs::Footstep& footstep)
  {
    if (footstep.dimensions.x == 0 &&
        footstep.dimensions.y == 0 &&
        footstep.dimensions.z == 0) {
      return std::max(minNotZero(minNotZero(footstep.dimensions.x,
                                            footstep.dimensions.y),
                                 footstep.dimensions.z),
                      0.1);
    }
    else {
      return std::max(minNotZero(minNotZero(depth_property_->getFloat(),
                                            width_property_->getFloat()),
                                 height_property_->getFloat()),
                      0.1);
        
    }
  }
  
  
  void FootstepDisplay::processMessage(const jsk_footstep_msgs::FootstepArray::ConstPtr& msg)
  {
    if (!validateFloats(*msg)) {
      setStatus(rviz::StatusProperty::Error, "Topic", "message contained invalid floating point values (nans or infs)");
      return;
    }
    latest_footstep_ = msg;
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if(!context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                    msg->header.stamp,
                                                   position, orientation)) {
      ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                 msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
      return;
    }

    // check thhe length of the shapes_
    allocateCubes(msg->footsteps.size());
    allocateTexts(msg->footsteps.size());
    line_->clear();
    line_->setLineWidth(0.01);
    line_->setNumLines(1);
    line_->setMaxPointsPerLine(1024);

    for (size_t i = 0; i < msg->footsteps.size(); i++)
    {
      ShapePtr shape = shapes_[i];
      rviz::MovableText* text = texts_[i];
      Ogre::SceneNode* node = text_nodes_[i];
      jsk_footstep_msgs::Footstep footstep = msg->footsteps[i];
      Ogre::Vector3 step_position;
      Ogre::Quaternion step_quaternion;
      if( !context_->getFrameManager()->transform( msg->header, footstep.pose,
                                                   step_position,
                                                   step_quaternion ))
      {
        ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                   qPrintable( getName() ), msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
        return;
      }
      shape->setPosition(step_position);
      shape->setOrientation(step_quaternion);
      // size of shape
      Ogre::Vector3 scale;
      if (footstep.dimensions.x == 0 &&
          footstep.dimensions.y == 0 &&
          footstep.dimensions.z == 0) {
        scale[0] = depth_;
        scale[1] = width_;
        scale[2] = height_;
      }
      else {
        scale[0] = footstep.dimensions.x;
        scale[1] = footstep.dimensions.y;
        scale[2] = footstep.dimensions.z;
      }
      shape->setScale(scale);      
      // update the size of text
      if (footstep.leg == jsk_footstep_msgs::Footstep::LEFT) {
        text->setCaption("L");
      }
      else if (footstep.leg == jsk_footstep_msgs::Footstep::RIGHT) {
        text->setCaption("R");
      }
      else {
        text->setCaption("unknown");
      }
      text->setCharacterHeight(estimateTextSize(footstep));
      node->setPosition(step_position);
      node->setOrientation(step_quaternion);
      text->setVisible(show_name_); // TODO
      line_->addPoint(step_position);
      
    }
    //updateFootstepSize();
    updateAlpha();
    context_->queueRender();
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::FootstepDisplay, rviz::Display )
