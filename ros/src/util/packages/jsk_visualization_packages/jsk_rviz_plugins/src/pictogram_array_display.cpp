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

#include "pictogram_array_display.h"

namespace jsk_rviz_plugins
{
  PictogramArrayDisplay::PictogramArrayDisplay()
  {
    setupFont();
  }

  PictogramArrayDisplay::~PictogramArrayDisplay()
  {
  }

  void PictogramArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  }

  void PictogramArrayDisplay::reset()
  {
    MFDClass::reset();
    for (size_t i = 0; i < pictograms_.size(); i++) {
      pictograms_[i]->setEnable(false);
    }
  }
  
  void PictogramArrayDisplay::onEnable()
  {
    subscribe();
    for (size_t i = 0; i < pictograms_.size(); i++) {
      pictograms_[i]->setEnable(false);
    }
  }

  void PictogramArrayDisplay::allocatePictograms(size_t num)
  {
    if (pictograms_.size() > num) {
      for (size_t i = num; i < pictograms_.size(); i++) {
        pictograms_[i]->setEnable(false);
      }
      pictograms_.resize(num);
    }
    else if (pictograms_.size() < num) {
      for (size_t i = pictograms_.size(); i < num; i++) {
        //pictograms_[i]->setEnable(false);
        PictogramObject::Ptr pictogram(new PictogramObject(scene_manager_,
                                                           scene_node_,
                                                           1.0));
        pictogram->setContext(context_);
        pictogram->setEnable(false);
        pictogram->start();
        pictogram->setColor(QColor(25, 255, 240));
        pictogram->setAlpha(1.0);
        pictograms_.push_back(pictogram);
      }
    }
  }
  
  void PictogramArrayDisplay::processMessage(
    const jsk_rviz_plugins::PictogramArray::ConstPtr& msg)
  {
    boost::mutex::scoped_lock (mutex_);
    allocatePictograms(msg->pictograms.size());
    for (size_t i = 0; i < pictograms_.size(); i++) {
      pictograms_[i]->setEnable(isEnabled());
    }
    if (!isEnabled()) {
      return;
    }
    for (size_t i = 0; i < msg->pictograms.size(); i++) {
      PictogramObject::Ptr pictogram = pictograms_[i];
      pictogram->setAction(msg->pictograms[i].action);
      if (msg->pictograms[i].action == jsk_rviz_plugins::Pictogram::DELETE) {
        continue;
      }
    
      if (msg->pictograms[i].size <= 0.0) {
        pictogram->setSize(0.5);
      }
      else {
        pictogram->setSize(msg->pictograms[i].size / 2.0);
      }
      pictogram->setColor(QColor(msg->pictograms[i].color.r * 255,
                                 msg->pictograms[i].color.g * 255,
                                 msg->pictograms[i].color.b * 255));
      pictogram->setAlpha(msg->pictograms[i].color.a);
      pictogram->setPose(msg->pictograms[i].pose,
                         msg->pictograms[i].header.frame_id);
      pictogram->setText(msg->pictograms[i].character);
      pictogram->setMode(msg->pictograms[i].mode);
      pictogram->setTTL(msg->pictograms[i].ttl);
      pictogram->setSpeed(msg->pictograms[i].speed);
    }
  }

  void PictogramArrayDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock (mutex_);
    for (size_t i = 0; i < pictograms_.size(); i++) {
      pictograms_[i]->update(wall_dt, ros_dt);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_rviz_plugins::PictogramArrayDisplay, rviz::Display);
