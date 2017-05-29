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


#include "jsk_topic_tools/vital_checker_nodelet.h"

namespace jsk_topic_tools
{
  void VitalCheckerNodelet::onInit()
  {
    DiagnosticNodelet::onInit();
    if (pnh_->hasParam("title")) {
      pnh_->getParam("title", title_);
    }
    else {
      NODELET_FATAL("no ~title is specified");
      return;
    }
    sub_ = pnh_->subscribe<topic_tools::ShapeShifter>(
      "input", 1,
      &VitalCheckerNodelet::inputCallback, this);
  }

  void VitalCheckerNodelet::subscribe()
  {
    
  }
  
  void VitalCheckerNodelet::unsubscribe()
  {

  }

  void VitalCheckerNodelet::inputCallback(
    const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
  }
  
  void VitalCheckerNodelet::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   title_ + " is running");
      stat.add("last alive time", vital_checker_->lastAliveTimeRelative());
    }
    else {
      addDiagnosticErrorSummary(
        title_, vital_checker_, stat);
    }
  }
}

#include <pluginlib/class_list_macros.h>
typedef jsk_topic_tools::VitalCheckerNodelet VitalCheckerNodelet;
PLUGINLIB_EXPORT_CLASS(VitalCheckerNodelet, nodelet::Nodelet)
