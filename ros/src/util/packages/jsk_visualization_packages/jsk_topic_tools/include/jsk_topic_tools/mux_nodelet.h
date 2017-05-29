/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#ifndef MUX_NODELET_H_
#define MUX_NODELET_H_

#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>

#include <topic_tools/MuxSelect.h>
#include <topic_tools/MuxAdd.h>
#include <topic_tools/MuxDelete.h>
#include <topic_tools/MuxList.h>

namespace jsk_topic_tools
{
  class MUX : public nodelet::Nodelet
  {
  public:
    typedef ros::MessageEvent<topic_tools::ShapeShifter> ShapeShifterEvent;
    virtual void onInit();
    virtual bool selectTopicCallback(topic_tools::MuxSelect::Request  &req,
                                     topic_tools::MuxSelect::Response &res);
    virtual bool addTopicCallback(topic_tools::MuxAdd::Request& req,
                                  topic_tools::MuxAdd::Response& res);
    virtual bool deleteTopicCallback(topic_tools::MuxDelete::Request& req,
                                     topic_tools::MuxDelete::Response& res);
    virtual bool listTopicCallback(topic_tools::MuxList::Request& req,
                                   topic_tools::MuxList::Response& res);
    virtual void inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
  protected:
    virtual void connectCb(const ros::SingleSubscriberPublisher& pub);
    virtual void subscribeSelectedTopic();
    bool advertised_;
    bool subscribing_;
    std::vector<std::string> topics_;
    std::string selected_topic_;
    boost::shared_ptr<ros::Subscriber> sub_;
    ros::Publisher pub_selected_;
    ros::Publisher pub_;
    ros::NodeHandle pnh_;
    ros::TransportHints th_;
    ros::ServiceServer ss_select_, ss_add_, ss_list_, ss_del_;
  };
}

#endif
