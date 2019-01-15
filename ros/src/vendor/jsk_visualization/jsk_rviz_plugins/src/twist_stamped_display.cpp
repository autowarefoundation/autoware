// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "twist_stamped_display.h"

namespace jsk_rviz_plugins
{
  TwistStampedDisplay::TwistStampedDisplay()
  {
    linear_scale_property_ = new rviz::FloatProperty("linear scale", 1.0,
                                                     "linear velocity scale",
                                                     this, SLOT(updateLinearScale()));
    angular_scale_property_ = new rviz::FloatProperty("angular scale", 1.0,
                                                      "angular velocity scale",
                                                     this, SLOT(updateAngularScale()));
    linear_color_property_ = new rviz::ColorProperty("linear color", QColor(0, 255, 0),
                                                     "linear velocity color",
                                                     this, SLOT(updateLinearColor()));
    angular_color_property_ = new rviz::ColorProperty("angular color", QColor(255, 0, 0),
                                                      "angular velocity color",
                                                     this, SLOT(updateAngularColor()));
    linear_scale_property_->setMin(0.0);
    angular_scale_property_->setMin(0.0);
  }

  TwistStampedDisplay::~TwistStampedDisplay()
  {
    delete linear_color_property_;
    delete angular_color_property_;
  }

  void TwistStampedDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    linear_arrow_.reset(new rviz::Arrow(scene_manager_, scene_node_));
    x_rotate_circle_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
    y_rotate_circle_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
    z_rotate_circle_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
    x_rotate_arrow_.reset(new rviz::Arrow(scene_manager_, scene_node_));
    y_rotate_arrow_.reset(new rviz::Arrow(scene_manager_, scene_node_));
    z_rotate_arrow_.reset(new rviz::Arrow(scene_manager_, scene_node_));
    updateLinearScale();
    updateAngularScale();
    updateLinearColor();
    updateAngularColor();
    Ogre::Vector3 zero_scale(0, 0, 0);
    linear_arrow_->setScale(zero_scale);
    x_rotate_arrow_->set(0, 0, 0, 0);
    y_rotate_arrow_->set(0, 0, 0, 0);
    z_rotate_arrow_->set(0, 0, 0, 0);
  }

  void TwistStampedDisplay::reset()
  {
    MFDClass::reset();
  }
  
  void TwistStampedDisplay::processMessage(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    // move scene_node_ to the frame pose
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(
         msg->header, position, orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    // linear velocity
    linear_arrow_->setColor(rviz::qtToOgre(linear_color_));
    Ogre::Vector3 linear_direction(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
    Ogre::Vector3 linear_scale(linear_scale_ * linear_direction.length(),
                               linear_scale_ * linear_direction.length(),
                               linear_scale_ * linear_direction.length());
    linear_arrow_->setScale(linear_scale);
    linear_arrow_->setDirection(linear_direction);

    // rotate velocity
    updateRotationVelocity(x_rotate_circle_,
                           x_rotate_arrow_,
                           Ogre::Vector3(0, 1, 0),
                           Ogre::Vector3(0, 0, 1),
                           Ogre::Vector3(1, 0, 0),
                           std::abs(msg->twist.angular.x),
                           msg->twist.angular.x > 0);
    updateRotationVelocity(y_rotate_circle_,
                           y_rotate_arrow_,
                           Ogre::Vector3(0, 0, 1),
                           Ogre::Vector3(1, 0, 0),
                           Ogre::Vector3(0, 1, 0),
                           std::abs(msg->twist.angular.y),
                           msg->twist.angular.y > 0);
    updateRotationVelocity(z_rotate_circle_,
                           z_rotate_arrow_,
                           Ogre::Vector3(1, 0, 0),
                           Ogre::Vector3(0, 1, 0),
                           Ogre::Vector3(0, 0, 1),
                           std::abs(msg->twist.angular.z),
                           msg->twist.angular.z > 0);
    Ogre::ColourValue c = rviz::qtToOgre(angular_color_);
    x_rotate_circle_->setColor(c.r, c.g, c.b, 1.0);
    y_rotate_circle_->setColor(c.r, c.g, c.b, 1.0);
    z_rotate_circle_->setColor(c.r, c.g, c.b, 1.0);
    x_rotate_arrow_->setColor(c);
    y_rotate_arrow_->setColor(c);
    z_rotate_arrow_->setColor(c);
  }

  void TwistStampedDisplay::updateRotationVelocity(
      BillboardLinePtr circle,
      ArrowPtr arrow,
      const Ogre::Vector3& ux,
      const Ogre::Vector3& uy,
      const Ogre::Vector3& uz,
      const double r,
      bool positive)
  {
    circle->clear();
    if (r < 1.0e-9) {           // too small to visualize it
      arrow->set(0, 0, 0, 0);
      return;
    }
    const double step = 10;     // per 10 deg
    const double start_theta = 20;
    const double end_theta = 340;
    circle->setMaxPointsPerLine((end_theta - start_theta) / step + 1); // +1?
    circle->setLineWidth(r * angular_scale_ / 2 * 0.1);
    for (double theta = start_theta; theta < end_theta; theta += step) {
      double rad = theta / 180 * M_PI;
      Ogre::Vector3 p = ux * cos(rad) * r * angular_scale_ + uy * sin(rad) * r * angular_scale_;
      circle->addPoint(p);
    }
    // put arrow
    if (positive) {
      double end_rad = (end_theta - step) / 180 * M_PI;
      Ogre::Vector3 endpoint = ux * cos(end_rad) * r * angular_scale_ + uy * sin(end_rad) * r * angular_scale_;
      Ogre::Vector3 direction = ux * (- sin(end_rad)) + uy * cos(end_rad);
      arrow->setPosition(endpoint);
      arrow->setDirection(direction);
    }
    else {
      double end_rad = (start_theta + step) / 180 * M_PI;
      Ogre::Vector3 endpoint = ux * cos(end_rad) * r * angular_scale_ + uy * sin(end_rad) * r * angular_scale_;
      Ogre::Vector3 direction = - ux * (- sin(end_rad)) - uy * cos(end_rad);
      arrow->setPosition(endpoint);
      arrow->setDirection(direction);
    }
    arrow->set(0, 0, r * angular_scale_ / 2, r * angular_scale_ / 2);
  }
  
  ////////////////////////////////////////////////////////
  // update methods
  ////////////////////////////////////////////////////////
  void TwistStampedDisplay::updateLinearScale()
  {
    linear_scale_ = linear_scale_property_->getFloat();
  }
  
  void TwistStampedDisplay::updateAngularScale()
  {
    angular_scale_ = angular_scale_property_->getFloat();
  }
  
  void TwistStampedDisplay::updateLinearColor()
  {
    linear_color_ = linear_color_property_->getColor();
  }
  
  void TwistStampedDisplay::updateAngularColor()
  {
    angular_color_ = angular_color_property_->getColor();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::TwistStampedDisplay, rviz::Display )
