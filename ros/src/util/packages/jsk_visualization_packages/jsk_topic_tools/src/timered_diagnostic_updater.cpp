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

#include "jsk_topic_tools/timered_diagnostic_updater.h"

namespace jsk_topic_tools
{
  TimeredDiagnosticUpdater::TimeredDiagnosticUpdater(
    ros::NodeHandle& nh,
    const ros::Duration& timer_duration):
    diagnostic_updater_(new diagnostic_updater::Updater)
  {
    timer_ = nh.createTimer(
      timer_duration, boost::bind(
        &TimeredDiagnosticUpdater::timerCallback,
        this,
        _1));
    timer_.stop();
  }
  
  void TimeredDiagnosticUpdater::start()
  {
    timer_.start();
  }

  TimeredDiagnosticUpdater::~TimeredDiagnosticUpdater()
  {
  }

  void TimeredDiagnosticUpdater::setHardwareID(const std::string& name)
  {
    diagnostic_updater_->setHardwareID(name);
  }
  
  void TimeredDiagnosticUpdater::add(const std::string& name,
                                     diagnostic_updater::TaskFunction f)
  {
    diagnostic_updater_->add(name, f);
  }
  
  // void TimeredDiagnosticUpdater::add(diagnostic_updater::DiagnosticTask task)
  // {
  //   diagnostic_updater_->add(task);
  // }

  void TimeredDiagnosticUpdater::update()
  {
    diagnostic_updater_->update();
  }
  
  void TimeredDiagnosticUpdater::timerCallback(const ros::TimerEvent& event)
  {
    update();
  }

}
