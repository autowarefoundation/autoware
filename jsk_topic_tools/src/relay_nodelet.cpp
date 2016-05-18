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
#include "jsk_topic_tools/relay_nodelet.h"

namespace jsk_topic_tools
{

  void Relay::onInit()
  {
    output_topic_name_ = "output";
    connection_status_ = NOT_INITIALIZED;
    pnh_ = getPrivateNodeHandle();
    // setup diagnostic
    diagnostic_updater_.reset(
      new TimeredDiagnosticUpdater(pnh_, ros::Duration(1.0)));
    diagnostic_updater_->setHardwareID(getName());
    diagnostic_updater_->add(
      getName() + "::Relay",
      boost::bind(
        &Relay::updateDiagnostic, this, _1));
    double vital_rate;
    pnh_.param("vital_rate", vital_rate, 1.0);
    vital_checker_.reset(
      new jsk_topic_tools::VitalChecker(1 / vital_rate));
    diagnostic_updater_->start();
    sub_ = pnh_.subscribe<topic_tools::ShapeShifter>(
      "input", 1,
      &Relay::inputCallback, this);
    change_output_topic_srv_ = pnh_.advertiseService(
      "change_output_topic", &Relay::changeOutputTopicCallback, this);
  }

  void Relay::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (connection_status_ == NOT_INITIALIZED) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                   "not initialized. Is "
                   + pnh_.resolveName("input") + " active?");
    }
    else if (connection_status_ == SUBSCRIBED) {
      if (vital_checker_->isAlive()) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     "subscribed: " + pnh_.resolveName("output"));
      }
      else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     "subscribed but no input. Is " 
                     + pnh_.resolveName("input") + " active?");
      }
      vital_checker_->registerStatInfo(stat);
    }
    else if (connection_status_ == NOT_SUBSCRIBED) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "not subscribed: " + pnh_.resolveName("output"));
    }
    stat.add("input topic", pnh_.resolveName("input"));
    stat.add("output topic", pnh_.resolveName("output"));
  }
  
  void Relay::inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (connection_status_ == NOT_INITIALIZED) {
      // this block is called only once
      // in order to advertise topic.
      // we need to subscribe at least one message
      // in order to detect message definition.
      pub_ = advertise(msg, output_topic_name_);
      connection_status_ = NOT_SUBSCRIBED;
      // shutdown subscriber
      sub_.shutdown();
      sample_msg_ = msg;
    }
    else if (pub_.getNumSubscribers() > 0) {
      vital_checker_->poke();
      pub_.publish(msg);
    }
  }

  void Relay::connectCb()
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("connectCB");
    if (connection_status_ != NOT_INITIALIZED) {
      if (pub_.getNumSubscribers() > 0) {
        if (connection_status_ == NOT_SUBSCRIBED) {
          NODELET_DEBUG("suscribe");
          sub_ = pnh_.subscribe<topic_tools::ShapeShifter>("input", 1,
                                                           &Relay::inputCallback, this);
          connection_status_ = SUBSCRIBED;
        }
      }
    }
  }

  void Relay::disconnectCb()
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("disconnectCb");
    if (connection_status_ != NOT_INITIALIZED) {
      if (pub_.getNumSubscribers() == 0) {
        if (connection_status_ == SUBSCRIBED) {
          NODELET_DEBUG("disconnect");
          sub_.shutdown();
          connection_status_ = NOT_SUBSCRIBED;
        }
      }
    }
  }

  ros::Publisher Relay::advertise(
    boost::shared_ptr<topic_tools::ShapeShifter const> msg,
    const std::string& topic)
  {
    ros::SubscriberStatusCallback connect_cb
      = boost::bind( &Relay::connectCb, this);
    ros::SubscriberStatusCallback disconnect_cb
      = boost::bind( &Relay::disconnectCb, this);
    ros::AdvertiseOptions opts(topic, 1,
                               msg->getMD5Sum(),
                               msg->getDataType(),
                               msg->getMessageDefinition(),
                               connect_cb,
                               disconnect_cb);
    opts.latch = false;
    return pnh_.advertise(opts);
  }
  
  bool Relay::changeOutputTopicCallback(
    jsk_topic_tools::ChangeTopic::Request &req,
    jsk_topic_tools::ChangeTopic::Response &res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    output_topic_name_ = req.topic;
    if (sample_msg_) {          // we can advertise it!
      pub_ = advertise(sample_msg_, output_topic_name_);
    }
    return true;
  }
}

typedef jsk_topic_tools::Relay Relay;
PLUGINLIB_EXPORT_CLASS(Relay, nodelet::Nodelet)
