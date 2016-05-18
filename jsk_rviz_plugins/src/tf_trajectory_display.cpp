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

#include "tf_trajectory_display.h"

namespace jsk_rviz_plugins
{
  TFTrajectoryDisplay::TFTrajectoryDisplay()
    : Display()
  {
    frame_property_ = new rviz::TfFrameProperty("frame", "",
                                                "frame to visualize trajectory",
                                                this,
                                                NULL,
                                                false,
                                                SLOT(updateFrame()));
    duration_property_ = new rviz::FloatProperty("duration", 10.0,
                                                 "duration to visualize trajectory",
                                                 this, SLOT(updateDuration()));
    line_width_property_ = new rviz::FloatProperty("line_width", 0.01,
                                                   "line width",
                                                   this, SLOT(updateLineWidth()));
    color_property_ = new rviz::ColorProperty("color", QColor(25, 255, 240),
                                              "color of trajectory",
                                              this, SLOT(updateColor()));
    duration_property_->setMin(0.0);
    line_width_property_->setMin(0.0);
  }

  TFTrajectoryDisplay::~TFTrajectoryDisplay()
  {
    delete line_width_property_;
    delete frame_property_;
    delete duration_property_;
    delete color_property_;
    delete line_;
  }

  void TFTrajectoryDisplay::onInitialize()
  {
    frame_property_->setFrameManager( context_->getFrameManager() );
    line_ = new rviz::BillboardLine(context_->getSceneManager(), scene_node_);
    updateFrame();
    updateDuration();
    updateColor();
    updateLineWidth();
  }

  void TFTrajectoryDisplay::updateFrame()
  {
    frame_ = frame_property_->getFrame().toStdString();
    trajectory_.clear();
  }

  void TFTrajectoryDisplay::updateDuration()
  {
    duration_ = duration_property_->getFloat();
  }

  void TFTrajectoryDisplay::updateColor()
  {
    color_ = color_property_->getColor();
  }

  void TFTrajectoryDisplay::onEnable()
  {
    line_->clear();
    trajectory_.clear();
  }

  void TFTrajectoryDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getFloat();
  }

  void TFTrajectoryDisplay::onDisable()
  {
    line_->clear();
    trajectory_.clear();
  }

  void TFTrajectoryDisplay::update(float wall_dt, float ros_dt)
  {
    if (frame_.empty()) {
      return;
    }
    std::string fixed_frame_id = context_->getFrameManager()->getFixedFrame();
    if (fixed_frame_ != fixed_frame_id) {
      fixed_frame_ = fixed_frame_id;
      line_->clear();
      trajectory_.clear();
      return;
    }
    fixed_frame_ = fixed_frame_id;
    ros::Time now = context_->getFrameManager()->getTime();
    std_msgs::Header header;
    header.stamp = ros::Time(0.0);
    header.frame_id = frame_;
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(
         header, position, orientation)) {
      setStatus(rviz::StatusProperty::Error, "transformation",
                (boost::format("Failed transforming from frame '%s' to frame '%s'")
                 % header.frame_id.c_str() % fixed_frame_id.c_str()).str().c_str());
      return;
    }
    setStatus(rviz::StatusProperty::Ok, "transformation", "Ok");
    geometry_msgs::PointStamped new_point;
    new_point.header.stamp = now;
    new_point.point.x = position[0];
    new_point.point.y = position[1];
    new_point.point.z = position[2];
    trajectory_.push_back(new_point);
    // check old data, is it too slow??
    for (std::vector<geometry_msgs::PointStamped>::iterator it = trajectory_.begin();
         it != trajectory_.end();) {
      ros::Duration duration = now - it->header.stamp;
      if (duration.toSec() > duration_) {
        it = trajectory_.erase(it);
      }
      else {
        break;
      }
    }
    line_->clear();
    line_->setNumLines(1);
    line_->setMaxPointsPerLine(trajectory_.size());
    line_->setLineWidth(line_width_);
    line_->setColor(color_.red() * 255.0, color_.green() * 255.0, color_.blue() * 255.0, 255.0);
    for (size_t i = 0; i < trajectory_.size(); i++) {
      Ogre::Vector3 p;
      p[0] = trajectory_[i].point.x;
      p[1] = trajectory_[i].point.y;
      p[2] = trajectory_[i].point.z;
      line_->addPoint(p);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::TFTrajectoryDisplay, rviz::Display )
