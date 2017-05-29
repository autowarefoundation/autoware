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


#ifndef JSK_TOPIC_TOOLS_SNAPSHOT_NODELET_H_
#define JSK_TOPIC_TOOLS_SNAPSHOT_NODELET_H_

#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <std_srvs/Empty.h>

namespace jsk_topic_tools
{
  class Snapshot: public nodelet::Nodelet
  {
  public:
    typedef ros::MessageEvent<topic_tools::ShapeShifter> ShapeShifterEvent;
    typedef boost::shared_ptr<Snapshot> Ptr;
  protected:
    virtual void onInit();
    virtual void inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
    virtual bool requestCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    
    ros::ServiceServer request_service_;
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Publisher pub_timestamp_;
    ros::Subscriber sub_;
    ros::NodeHandle pnh_;
    bool subscribing_;
    bool advertised_;
    bool requested_;
    bool latch_;
  private:
    
  };
}

#endif
