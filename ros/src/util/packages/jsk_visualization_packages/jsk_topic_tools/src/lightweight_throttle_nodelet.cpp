/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, JSK Lab
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

#include <pluginlib/class_list_macros.h>
#include "jsk_topic_tools/lightweight_throttle_nodelet.h"

namespace jsk_topic_tools
{
  void LightweightThrottle::onInit()
  {
    pnh_ = this->getPrivateNodeHandle();
    latest_stamp_ = ros::Time::now();
    advertised_ = false;
    subscribing_ = false;
    pnh_.param("update_rate", update_rate_, 1.0); // default 1.0
    // Subscribe input topic at first in order to decide
    // message type of publisher.
    // nodelet will unsubscribe input topic after it receives the first topic.
    sub_.reset(new ros::Subscriber(
                 pnh_.subscribe<topic_tools::ShapeShifter>("input", 1,
                                                           &LightweightThrottle::inCallback,
                                                           this,
                                                           th_)));
  }

  void LightweightThrottle::connectionCallback(
    const ros::SingleSubscriberPublisher& pub)
  {
    if (pub_.getNumSubscribers() > 0) {
      if (!subscribing_) {
        sub_.reset(new ros::Subscriber(
                     pnh_.subscribe<topic_tools::ShapeShifter>(
                       "input", 1,
                       &LightweightThrottle::inCallback,
                       this,
                       th_)));
        subscribing_ = true;
      }
    }
    else {      // No subscribers, nodelet can unsubscribe input topic
      if (subscribing_) {
        sub_->shutdown();
        subscribing_ = false;
      }
    }
  }
  
  void LightweightThrottle::inCallback(
    const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    // advertise if not
    if (!advertised_) {
      // This section should be called once
      sub_->shutdown();         // Shutdown before advertising topic
      ros::SubscriberStatusCallback connect_cb
        = boost::bind(&LightweightThrottle::connectionCallback, this, _1);
      ros::AdvertiseOptions opts("output", 1,
                                 msg->getMD5Sum(),
                                 msg->getDataType(),
                                 msg->getMessageDefinition(),
                                 connect_cb,
                                 connect_cb);
      advertised_ = true;
      pub_ = pnh_.advertise(opts);
    }
    ros::Time now = ros::Time::now();
    if ((now - latest_stamp_).toSec() > 1 / update_rate_) {
      pub_.publish(msg);
      latest_stamp_ = now;
    }
  }
}

typedef jsk_topic_tools::LightweightThrottle LightweightThrottle;
PLUGINLIB_EXPORT_CLASS(LightweightThrottle, nodelet::Nodelet)
