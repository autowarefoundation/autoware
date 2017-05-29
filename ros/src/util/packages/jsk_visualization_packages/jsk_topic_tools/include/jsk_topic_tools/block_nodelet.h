// -*- mode: c++ -*-
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

#ifndef BLOCK_NODELET_H_
#define BLOCK_NODELET_H_

#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>

namespace jsk_topic_tools
{

  /***************************************
   *
   *
   *  [input] -- |  | -- [input_original] -- |process| -- [output_original] -- |  | -- [output]
   *
   * * a subscriber to subscribe input := sub_input_
   * * a subscriber to subscribe output_original := sub_output_original_
   * * a publisher to publish input_original := pub_input_original_
   * * a publisher to publish output := pub_output_
   */
  
  class Block : public nodelet::Nodelet
  {
  public:
    typedef ros::MessageEvent<topic_tools::ShapeShifter> ShapeShifterEvent;
  protected:
    virtual void onInit();

    
    ros::NodeHandle pnh_;
    bool pub_input_original_advertised_, pub_output_advertised_;
    bool sub_input_subscribing_, sub_output_original_subscribing_;
    ros::Subscriber sub_input_, sub_output_original_;
    ros::Publisher pub_input_original_, pub_output_;
    double check_rate_;        // in Hz
    virtual void inputCallback(
      const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
    virtual void outputOriginalCallback(
      const boost::shared_ptr<topic_tools::ShapeShifter const>& msg);
    virtual void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
  };
}

#endif
