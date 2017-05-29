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
#include <pluginlib/class_list_macros.h>
#include "jsk_topic_tools/passthrough_nodelet.h"

namespace jsk_topic_tools
{
  /**
   * senario 1.
   *   1-1 subscribe input
   *   1-2 message
   *   1-3 advertise output
   *   1-4 unsubscribe input
   *   1-5 prepare for request
   * senario 2.
   *   1-1 subscribe input
   *   1-2 request
   *   1-3 message
   *   1-4 advertise
   *   1-5 unsubscribe at the end of request
   */
  void Passthrough::onInit()
  {
    advertised_ = false;
    publish_requested_ = false;
    pnh_ = getPrivateNodeHandle();
    subscribing_ = true;
    pnh_.param("default_duration", default_duration_, 10.0);
    sub_ = pnh_.subscribe<topic_tools::ShapeShifter>(
      "input", 1,
      &Passthrough::inputCallback, this);
    request_duration_srv_ = pnh_.advertiseService(
      "request_duration", &Passthrough::requestDurationCallback, this);
    request_srv_ = pnh_.advertiseService(
      "request", &Passthrough::requestCallback, this);
    stop_srv_ = pnh_.advertiseService(
      "stop", &Passthrough::stopCallback, this);
  }

  bool Passthrough::stopCallback(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // force to stop publishing
    if (!publish_requested_) {
      NODELET_DEBUG("already stoppped");
    }
    publish_requested_ = false;
    return true;
  }

  void Passthrough::requestDurationCallbackImpl(
    const ros::Duration& duration)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // special case of ros::Duration(0), it means eternal
    if (duration == ros::Duration(0)) {
      end_time_ = ros::Time(0);
      publish_requested_ = true;
    }
    else {
      ros::Time now = ros::Time::now();
      if (publish_requested_) {
        // check need to update end_time or not
        if (end_time_ < now + duration) {
          end_time_ = now + duration;
        }
      }
      else {
        publish_requested_ = true;
        end_time_ = now + duration;
      }
    }
    if (!subscribing_) {
      NODELET_DEBUG("suscribe");
      sub_ = pnh_.subscribe<topic_tools::ShapeShifter>(
        "input", 1,
        &Passthrough::inputCallback, this);
      subscribing_ = true;
    }
  }
  
  bool Passthrough::requestDurationCallback(
    jsk_topic_tools::PassthroughDuration::Request &req,
    jsk_topic_tools::PassthroughDuration::Response &res)
  {
    requestDurationCallbackImpl(req.duration);
    return true;
  }

  bool Passthrough::requestCallback(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &res)
  {
    requestDurationCallbackImpl(ros::Duration(default_duration_));
    return true;
  }
  
  void Passthrough::inputCallback(
    const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!advertised_) {
      // this block is called only once
      // in order to advertise topic.
      // we need to subscribe at least one message
      // in order to detect message definition.
      pub_ = advertise(msg, "output");
      advertised_ = true;
    }
    if (publish_requested_) {
      ros::Time now = ros::Time::now();
      if (end_time_ == ros::Time(0) || // ros::Time(0) means eternal publishing
          end_time_ > now) {
        pub_.publish(msg);
      }
      // check it goes over end_time_ 
      if (end_time_ != ros::Time(0) && end_time_ < now) {
        publish_requested_ = false;
      }
    }
    if (!publish_requested_) {
      // Unsubscribe input anyway
      sub_.shutdown();
      subscribing_ = false;
    }
  }

  void Passthrough::connectCb()
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("connectCB");
    if (advertised_) {
      if (pub_.getNumSubscribers() > 0) {
        if (!subscribing_ && publish_requested_) {
          NODELET_DEBUG("suscribe");
          sub_ = pnh_.subscribe<topic_tools::ShapeShifter>(
            "input", 1,
            &Passthrough::inputCallback, this);
          subscribing_ = true;
        }
      }
    }
  }

  void Passthrough::disconnectCb()
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("disconnectCb");
    if (advertised_) {
      if (pub_.getNumSubscribers() == 0) {
        if (subscribing_) {
          NODELET_DEBUG("disconnect");
          sub_.shutdown();
          subscribing_ = false;
        }
      }
    }
  }

  
  ros::Publisher Passthrough::advertise(
    boost::shared_ptr<topic_tools::ShapeShifter const> msg,
    const std::string& topic)
  {
    ros::SubscriberStatusCallback connect_cb
      = boost::bind( &Passthrough::connectCb, this);
    ros::SubscriberStatusCallback disconnect_cb
      = boost::bind( &Passthrough::disconnectCb, this);
    ros::AdvertiseOptions opts(topic, 1,
                               msg->getMD5Sum(),
                               msg->getDataType(),
                               msg->getMessageDefinition());
    opts.latch = true;
    return pnh_.advertise(opts);
  }
}

typedef jsk_topic_tools::Passthrough Passthrough;
PLUGINLIB_EXPORT_CLASS(Passthrough, nodelet::Nodelet)
