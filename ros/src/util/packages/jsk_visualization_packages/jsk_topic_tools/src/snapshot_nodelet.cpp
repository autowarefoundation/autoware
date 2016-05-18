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

#include "jsk_topic_tools/snapshot_nodelet.h"
#include <std_msgs/Time.h>

namespace jsk_topic_tools
{
  void Snapshot::onInit()
  {
    advertised_ = false;
    subscribing_ = false;
    pnh_ = getPrivateNodeHandle();
    pnh_.param("latch", latch_, false);
    pub_timestamp_ = pnh_.advertise<std_msgs::Time>("output/stamp", 1);
    sub_ = pnh_.subscribe<topic_tools::ShapeShifter>(
      "input", 1,
      &Snapshot::inputCallback, this);
    request_service_ = pnh_.advertiseService(
      "request",
      &Snapshot::requestCallback, this);
  }

  void Snapshot::inputCallback(
    const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!advertised_) {         // initialization
      ros::AdvertiseOptions opts("output", 1,
                                 msg->getMD5Sum(),
                                 msg->getDataType(),
                                 msg->getMessageDefinition());
      opts.latch = latch_;
      pub_ = pnh_.advertise(opts);
      advertised_ = true;
      if (requested_) {
        pub_.publish(msg);
        std_msgs::Time timestamp;
        timestamp.data = ros::Time::now();
        pub_timestamp_.publish(timestamp);
        requested_ = false;
      }
      sub_.shutdown();
    }
    else {
      if (requested_) {
        pub_.publish(msg);
        std_msgs::Time timestamp;
        timestamp.data = ros::Time::now();
        pub_timestamp_.publish(timestamp);
        requested_ = false;
        sub_.shutdown();
      }
    }
  }


  bool Snapshot::requestCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    requested_ = true;
    sub_ = pnh_.subscribe<topic_tools::ShapeShifter>(
      "input", 1,
      &Snapshot::inputCallback, this);
    return true;
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_topic_tools::Snapshot, nodelet::Nodelet);
