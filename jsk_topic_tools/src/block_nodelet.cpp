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
#include "jsk_topic_tools/block_nodelet.h"

#include <ros/advertise_options.h>

namespace jsk_topic_tools
{

  
  
  void Block::onInit()
  {
    pnh_ = getPrivateNodeHandle();
    pub_input_original_advertised_ = pub_output_advertised_ = false;
    sub_input_subscribing_ = sub_output_original_subscribing_ = false;
    pnh_.param("check_rate", check_rate_, 1.0); // defaults to 1Hz
    sub_input_
      = pnh_.subscribe<topic_tools::ShapeShifter>("input", 1,
                                                  &Block::inputCallback,
                                                  this);
    sub_output_original_
      = pnh_.subscribe<topic_tools::ShapeShifter>(
        "output_original", 1,
        &Block::outputOriginalCallback,
        this);
    timer_ = pnh_.createTimer(ros::Duration(1 / check_rate_),
                             &Block::timerCallback, this);
  }

  void Block::timerCallback(const ros::TimerEvent& event)
  {
    NODELET_DEBUG("timerCallback");
    if (pub_input_original_advertised_ && pub_output_advertised_) {
      // if the publishers are not advertised, we need to wait until
      // the subscribers get the first messages

      // next, check the number of the publisher
      if (pub_output_.getNumSubscribers() > 0) {
        NODELET_DEBUG("subscribe input");
        // we need to run all the subscribers
        if (!sub_input_subscribing_) {
          sub_input_ = pnh_.subscribe<topic_tools::ShapeShifter>(
            "input", 1,
            &Block::inputCallback,
            this);
          sub_input_subscribing_ = true;
        }
        if (!sub_output_original_subscribing_) {
          NODELET_DEBUG("subscribe output original");
          sub_output_original_
            = pnh_.subscribe<topic_tools::ShapeShifter>(
              "output_original", 1,
              &Block::outputOriginalCallback,
              this);
          sub_output_original_subscribing_ = true;
        }
      }
      else {
        // we need to shutdown the subscribers
        if (sub_input_subscribing_) {
          NODELET_DEBUG("shutdown input");
          sub_input_.shutdown();
          sub_input_subscribing_ = false;
        }
        if (sub_output_original_subscribing_) {
          NODELET_DEBUG("shutdown output_original");
          sub_output_original_.shutdown();
          sub_output_original_subscribing_ = false;
        }
      }
    }
  }
  
  void Block::inputCallback(
    const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    NODELET_DEBUG("inputCallback");
    // first we need to check is the publisher is advertised or not
    if (!pub_input_original_advertised_) {
      NODELET_DEBUG("advertise input_original");
      pub_input_original_ = msg->advertise(pnh_, "input_original", 1);
      pub_input_original_advertised_ = true;
      // shutdown subscriber
      if (pub_output_advertised_) {
        NODELET_DEBUG("shutdown input");
        sub_input_.shutdown();
      }
      else {
        NODELET_DEBUG("publish input_original");
        pub_input_original_.publish(msg);
      }
    }
    else {
      NODELET_DEBUG("publish input_original");
      pub_input_original_.publish(msg);
    }
  }

  void Block::outputOriginalCallback(
    const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
  {
    NODELET_DEBUG("outputOriginalCallback");
    // first we need to check is the publisher is advertised or not
    if (!pub_output_advertised_) {
      NODELET_DEBUG("advertise output");;
      pub_output_ = msg->advertise(pnh_, "output", 1);
      pub_output_advertised_ = true;
      // shutdown subscriber
      sub_output_original_.shutdown();
      if (pub_input_original_advertised_) {
        NODELET_DEBUG("shutdown input");
        sub_input_.shutdown();
      }
      else {
        NODELET_DEBUG("publish output");
        pub_output_.publish(msg);
      }
    }
    else {
      NODELET_DEBUG("publish output");
      pub_output_.publish(msg);
    }
  }  
}

typedef jsk_topic_tools::Block Block;
PLUGINLIB_EXPORT_CLASS(Block, nodelet::Nodelet)

