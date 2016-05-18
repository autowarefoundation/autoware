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

#include "jsk_topic_tools/vital_checker.h"

namespace jsk_topic_tools
{
  VitalChecker::VitalChecker(const double dead_sec):
    dead_sec_(dead_sec)
  {

  }

  VitalChecker::~VitalChecker()
  {

  }

  void VitalChecker::poke()
  {
    boost::mutex::scoped_lock lock(mutex_);
    last_alive_time_ = ros::Time::now();
  }

  double VitalChecker::lastAliveTimeRelative()
  {
    return (ros::Time::now() - last_alive_time_).toSec();
  }
  
  bool VitalChecker::isAlive()
  {
    bool ret;
    {
     boost::mutex::scoped_lock lock(mutex_);
     ret = (ros::Time::now() - last_alive_time_).toSec() < dead_sec_;
    }
    return ret;
  }

  double VitalChecker::deadSec()
  {
    return dead_sec_;
  }

  void VitalChecker::registerStatInfo(diagnostic_updater::DiagnosticStatusWrapper &stat, const std::string& prefix)
  {
    boost::mutex::scoped_lock lock(mutex_);
    stat.add(prefix, 
             (boost::format("%f sec before") % lastAliveTimeRelative()).str());
  }

}
