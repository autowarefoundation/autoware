// -*- mode: c++ -*-
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

#ifndef RELAY_NODELET_H_
#define RELAY_NODELET_H_

#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <jsk_topic_tools/ChangeTopic.h>
#include "jsk_topic_tools/connection_based_nodelet.h"
#include "jsk_topic_tools/timered_diagnostic_updater.h"
#include "jsk_topic_tools/diagnostic_utils.h"

namespace jsk_topic_tools
{
  class Relay : public nodelet::Nodelet
  {
  public:
    typedef ros::MessageEvent<topic_tools::ShapeShifter> ShapeShifterEvent;
    virtual void onInit();
    virtual void inputCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
  protected:
    virtual void connectCb();
    virtual void disconnectCb();
    virtual bool changeOutputTopicCallback(
      jsk_topic_tools::ChangeTopic::Request &req,
      jsk_topic_tools::ChangeTopic::Response &res);
    virtual ros::Publisher advertise(
      boost::shared_ptr<topic_tools::ShapeShifter const> msg,
      const std::string& topic);
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    boost::shared_ptr<topic_tools::ShapeShifter const> sample_msg_;
    std::string output_topic_name_;
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ConnectionStatus connection_status_;
    ros::NodeHandle pnh_;
    ros::ServiceServer change_output_topic_srv_;
    /** @brief
     * Pointer to TimeredDiagnosticUpdater to call
     * updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper&)
     * periodically.
     */
    TimeredDiagnosticUpdater::Ptr diagnostic_updater_;
    VitalChecker::Ptr vital_checker_;
  };
}

#endif
