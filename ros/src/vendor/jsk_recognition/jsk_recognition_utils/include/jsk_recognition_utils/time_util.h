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

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <std_msgs/Float32.h>

namespace jsk_recognition_utils
{
  class WallDurationTimer;
  
  class ScopedWallDurationReporter
  {
  public:
    typedef boost::shared_ptr<ScopedWallDurationReporter> Ptr;
    ScopedWallDurationReporter(WallDurationTimer* parent);
    ScopedWallDurationReporter(WallDurationTimer* parent,
                               ros::Publisher& pub_latest,
                               ros::Publisher& pub_average);
    virtual ~ScopedWallDurationReporter();
    virtual void setIsPublish(bool);
    virtual void setIsEnabled(bool);
  protected:
    WallDurationTimer* parent_;
    ros::WallTime start_time_;
    ros::Publisher pub_latest_, pub_average_;
    bool is_publish_;
    bool is_enabled_;
  private:
    
  };
  
  class WallDurationTimer
  {
  public:
    typedef boost::shared_ptr<WallDurationTimer> Ptr;
    WallDurationTimer(const int max_num);
    virtual void report(ros::WallDuration& duration);
    virtual ScopedWallDurationReporter reporter();
    virtual ScopedWallDurationReporter reporter(
      ros::Publisher& pub_latest,
      ros::Publisher& pub_average);
    virtual void clearBuffer();
    virtual double meanSec();
    virtual double latestSec();
    virtual size_t sampleNum();
  protected:
    const int max_num_;
    boost::circular_buffer<ros::WallDuration> buffer_;
  private:
  };
  
}
