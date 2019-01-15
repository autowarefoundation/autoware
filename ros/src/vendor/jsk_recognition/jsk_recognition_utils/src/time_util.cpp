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

#include "jsk_recognition_utils/time_util.h"
#include <std_msgs/Float32.h>

namespace jsk_recognition_utils
{
  ScopedWallDurationReporter::ScopedWallDurationReporter(WallDurationTimer* parent):
    parent_(parent), start_time_(ros::WallTime::now()),
    is_publish_(false), is_enabled_(true)
  {

  }

  ScopedWallDurationReporter::ScopedWallDurationReporter(
    WallDurationTimer* parent,
    ros::Publisher& pub_latest,
    ros::Publisher& pub_average):
    parent_(parent), start_time_(ros::WallTime::now()),
    pub_latest_(pub_latest), pub_average_(pub_average),
    is_publish_(true), is_enabled_(true)
  {

  }
  
  ScopedWallDurationReporter::~ScopedWallDurationReporter()
  {
    ros::WallTime end_time = ros::WallTime::now();
    ros::WallDuration d = end_time - start_time_;
    if (is_enabled_) {
      parent_->report(d);
      if (is_publish_) {
        std_msgs::Float32 ros_latest;
        ros_latest.data = parent_->latestSec();
        pub_latest_.publish(ros_latest);
        std_msgs::Float32 ros_average;
        ros_average.data = parent_->meanSec();
        pub_average_.publish(ros_average);
      }
    }
  }

  void ScopedWallDurationReporter::setIsPublish(bool v)
  {
    is_publish_ = v;
  }

  void ScopedWallDurationReporter::setIsEnabled(bool v)
  {
    is_enabled_ = v;
  }
  
  WallDurationTimer::WallDurationTimer(const int max_num):
    max_num_(max_num), buffer_(max_num)
  {
  }
  
  void WallDurationTimer::report(ros::WallDuration& duration)
  {
    buffer_.push_back(duration);
  }

  ScopedWallDurationReporter WallDurationTimer::reporter()
  {
    return ScopedWallDurationReporter(this);
  }

  ScopedWallDurationReporter WallDurationTimer::reporter(
    ros::Publisher& pub_latest,
    ros::Publisher& pub_average)
  {
    return ScopedWallDurationReporter(this, pub_latest, pub_average);
  }

  double WallDurationTimer::latestSec()
  {
    return buffer_[buffer_.size() - 1].toSec();
  }
  
  void WallDurationTimer::clearBuffer()
  {
    buffer_.clear();
  }

  double WallDurationTimer::meanSec()
  {
    double secs = 0.0;
    for (size_t i = 0; i < buffer_.size(); i++) {
      secs += buffer_[i].toSec();
    }
    return secs / buffer_.size();
  }

  size_t WallDurationTimer::sampleNum()
  {
    return buffer_.size();
  }
}
