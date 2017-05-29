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

#include <jsk_topic_tools/log_utils.h>
#include "jsk_recognition_utils/tf_listener_singleton.h"
#include <boost/format.hpp>

namespace jsk_recognition_utils
{
  tf::TransformListener* TfListenerSingleton::getInstance()
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!instance_) {
      JSK_ROS_INFO("instantiating tf::TransformListener");
      instance_ = new tf::TransformListener(ros::Duration(30.0));
    }
    return instance_;
  }

  void TfListenerSingleton::destroy()
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (instance_) {
      delete instance_;
    }
  }

  tf::StampedTransform lookupTransformWithDuration(
    tf::TransformListener* listener,
    const std::string& to_frame,
    const std::string& from_frame,
    const ros::Time& stamp,
    ros::Duration duration)
  {
    if (listener->waitForTransform(from_frame, to_frame, stamp, duration)) {
      tf::StampedTransform transform;
      listener->lookupTransform(
        from_frame, to_frame, stamp, transform);
      return transform;
    }
    throw tf2::TransformException(
      (boost::format("Failed to lookup transformation from %s to %s")
       % from_frame.c_str() % to_frame.c_str()).str().c_str());
      
  }
  
  tf::TransformListener* TfListenerSingleton::instance_;
  boost::mutex TfListenerSingleton::mutex_;
}
