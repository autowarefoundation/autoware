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
#include "jsk_topic_tools/mux_nodelet.h"
#include <std_msgs/String.h>
#include "jsk_topic_tools/rosparam_utils.h"

namespace jsk_topic_tools
{

  const static std::string g_none_topic = "__none";
  
  void MUX::onInit()
  {
    advertised_ = false;
    pnh_ = getPrivateNodeHandle();
    readVectorParameter(pnh_, "topics", topics_);
    if (topics_.size() < 1) {
      NODELET_FATAL("need to specify at least one topic in ~topics");
      return;
    }
    pub_selected_ = pnh_.advertise<std_msgs::String>("selected", 1, true);
    // in original mux node, it subscribes all the topics first, however
    // in our version, we never subscribe topic which are not selected.
    // ros::SubscriberStatusCallback connect_cb
    //     = boost::bind( &Mux::connectCb, this);
    selected_topic_ = topics_[0];
    subscribeSelectedTopic();
    // service advertise: _select, select, add, list, delete
    ss_select_ = pnh_.advertiseService("select", &MUX::selectTopicCallback, this);
    ss_add_ = pnh_.advertiseService("add", &MUX::addTopicCallback, this);
    ss_list_ = pnh_.advertiseService("list_topics", &MUX::listTopicCallback, this);
    ss_del_ = pnh_.advertiseService("delete", &MUX::deleteTopicCallback, this);
    
  }

  void MUX::connectCb(const ros::SingleSubscriberPublisher& pub)
  {
    // new subscriber come or
    if (pub_.getNumSubscribers() > 0) {
      // subscribe topic again
      if (!subscribing_) {
        //NODELET_INFO("subscribe");
        sub_.reset(new ros::Subscriber(
                     pnh_.subscribe<topic_tools::ShapeShifter>(
                       selected_topic_, 10,
                       &MUX::inputCallback, this, th_)));
        subscribing_ = true;
      }
    }
    else {
      if (subscribing_) {
        //NODELET_INFO("unsubscribe");
        sub_->shutdown();
        subscribing_ = false;
      }
    }
  }

  
  
  bool MUX::selectTopicCallback(topic_tools::MuxSelect::Request  &req,
                                topic_tools::MuxSelect::Response &res)
  {
    res.prev_topic = selected_topic_;
    if (selected_topic_ != g_none_topic) {
      //NODELET_INFO("unsubscribe");
      sub_->shutdown();            // unsubscribe first
    }

    if (req.topic == g_none_topic) { // same topic
      selected_topic_ = g_none_topic;
      return true;
    }
    for (size_t i = 0; i < topics_.size(); i++) {
      if (pnh_.resolveName(topics_[i]) == pnh_.resolveName(req.topic)) {
        // subscribe the topic
        selected_topic_ = topics_[i];
        subscribeSelectedTopic();
        return true;
      }
    }

    NODELET_WARN("%s is not provided in topic list", req.topic.c_str());
    return false;
  }

  bool MUX::addTopicCallback(topic_tools::MuxAdd::Request& req,
                        topic_tools::MuxAdd::Response& res)
  {
    NODELET_INFO("trying to add %s to mux", req.topic.c_str());
    if (req.topic == g_none_topic) {
      NODELET_WARN("failed to add topic %s to mux, because it's reserved for special use",
                   req.topic.c_str());
      return false;
    }
    
    for (size_t i = 0; i < topics_.size(); i++) {
      if (pnh_.resolveName(topics_[i]) == pnh_.resolveName(req.topic)) {
        NODELET_WARN("tried to add a topic that mux was already listening to: [%s]", 
                     topics_[i].c_str());
        return false;
      }
    }

    // in original mux, it subscribes the topic immediately after adds topic.
    // in this version, we postpone the subscription until selected.
    
    topics_.push_back(ros::names::resolve(req.topic));
    return true;
  }

  bool MUX::deleteTopicCallback(topic_tools::MuxDelete::Request& req,
                                topic_tools::MuxDelete::Response& res)
  {
    // cannot delete the topic now selected
    for (size_t i = 0; i < topics_.size(); i++) {
      if (pnh_.resolveName(topics_[i]) == pnh_.resolveName(req.topic)) {
        if (pnh_.resolveName(req.topic) == pnh_.resolveName(selected_topic_)) {
          NODELET_WARN("tried to delete currently selected topic %s from mux",
                       req.topic.c_str());
          return false;
        }
        topics_.erase(topics_.begin() + i);
        return true;
      }
    }
    NODELET_WARN("cannot find the topics %s in the list of mux",
                 req.topic.c_str());
    return false;
  }

  bool MUX::listTopicCallback(topic_tools::MuxList::Request& req,
                              topic_tools::MuxList::Response& res)
  {
    for (size_t i = 0; i < topics_.size(); i++) {
      res.topics.push_back(pnh_.resolveName(topics_[i]));
    }
    return true;
  }
  
  void MUX::subscribeSelectedTopic()
  {
    // subscribe topic in order to "advertise"
    advertised_ = false;
    subscribing_ = false;
    // assume that selected_topic_ is already set correctly
    if (selected_topic_ == g_none_topic) {
      NODELET_WARN("none topic is selected");
      return;
    }
    //NODELET_INFO("subscribe");
    sub_.reset(new ros::Subscriber(
                 pnh_.subscribe<topic_tools::ShapeShifter>(
                   selected_topic_, 10,
                   &MUX::inputCallback, this, th_)));
    std_msgs::String msg;
    msg.data = selected_topic_;
    pub_selected_.publish(msg);
  }

  void MUX::inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    if (!advertised_) {         // first time
      ros::SubscriberStatusCallback connect_cb
        = boost::bind(&MUX::connectCb, this, _1);
      ros::AdvertiseOptions opts("output", 1,
                                 msg->getMD5Sum(),
                                 msg->getDataType(),
                                 msg->getMessageDefinition(),
                                 connect_cb,
                                 connect_cb);
      pub_ = pnh_.advertise(opts);
      advertised_ = true;
      sub_->shutdown();
      //NODELET_INFO("unsubscribe");
    }
    pub_.publish(msg);
  }
  
}

typedef jsk_topic_tools::MUX MUX;
PLUGINLIB_EXPORT_CLASS(MUX, nodelet::Nodelet)

