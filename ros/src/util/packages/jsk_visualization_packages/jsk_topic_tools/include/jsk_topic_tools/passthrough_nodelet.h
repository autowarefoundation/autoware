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

#ifndef PASSTHROUGH_NODELET_H_
#define PASSTHROUGH_NODELET_H_

#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <jsk_topic_tools/PassthroughDuration.h>
#include <std_srvs/Empty.h>

namespace jsk_topic_tools
{
  class Passthrough : public nodelet::Nodelet
  {
  public:
    typedef ros::MessageEvent<topic_tools::ShapeShifter> ShapeShifterEvent;
  protected:
    /** @brief
     *  Initialization function.
     *
     *  Setup subscriber for ~input topic.
     */
    virtual void onInit();

    /** @brief
     *  Callback function for ~input.
     *  It advertise ~output topic according to the definition of input topic
     *  at the first time.
     *  After that, Passthrough relays ~input topic to ~output topic
     *  until end_time_.
     *
     *  If ros::Time::now() goes over end_time_, publish_requested_ flag is set
     *  to false.
     */
    virtual void inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
    
    /** @brief
     *  Callback function for ~request service.
     *
     *  This callback reqtrieves std_srvs::Empty and
     *  enables passthrough for ~default_duration seconds.
     */
    virtual bool requestCallback(
      std_srvs::Empty::Request &req,
      std_srvs::Empty::Response &res);
    
    /** @brief
     *  Callback function for ~request_duration service.
     *
     *  This callback reqtrieves jsk_topic_tools::PassThroughDuration and
     *  it has a duration to relay message.
     *  In side of this function, end_time_ is updated relative to ros::Time::now().
     *
     *  If duration is equivalent to ros::Duration(0), it is interpreted as eternal.
     */
    virtual bool requestDurationCallback(
      jsk_topic_tools::PassthroughDuration::Request &req,
      jsk_topic_tools::PassthroughDuration::Response &res);

    /** @brief
     *  Implementation of requestDurationCallback.
     *
     */
    virtual void requestDurationCallbackImpl(
      const ros::Duration& duration);
    
    /** @brief
     * Advertise a topic according to a specific message instance.
     */
    virtual ros::Publisher advertise(
      boost::shared_ptr<topic_tools::ShapeShifter const> msg,
      const std::string& topic);

    /** @brief
     * Callback function of ~stop service.
     * Force to stop publishing messages.
     */
    virtual bool stopCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);

    virtual void disconnectCb();
    virtual void connectCb();

    ros::Time finish_time_;
    bool publish_requested_;
    double default_duration_;
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    bool advertised_;
    bool subscribing_;
    ros::NodeHandle pnh_;
    ros::Time end_time_;
    ros::ServiceServer request_duration_srv_;
    ros::ServiceServer stop_srv_;
    ros::ServiceServer request_srv_;
  };
}

#endif
