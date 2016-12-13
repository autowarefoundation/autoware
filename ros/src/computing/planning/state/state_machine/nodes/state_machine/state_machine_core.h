/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef STATE_MACHINE_CORE_H
#define STATE_MACHINE_CORE_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Int32.h>

// User Defined includes
#include "runtime_manager/traffic_light.h"
#include "state_machine.h"

namespace state_machine
{
class StateMachineNode
{
public:

  StateMachineNode();
  ~StateMachineNode();

  void run();

private:

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // State context class
  StateContext sc_;

  // publisher
  ros::Publisher pub_;

  // subscriber
  ros::Subscriber sub1_,sub2_;

  // variables
  bool is_manual_light_detection_;

  // callbacks
  void callbackFromLightColor(const runtime_manager::traffic_lightConstPtr &msg);
  void callbackFromLightColorManaged(const runtime_manager::traffic_lightConstPtr &msg);

  // initializer
  void initForROS();

  // functions
  void publish() const;
};
} // state_machine
#endif  // STATE_MACHINE_CORE_H
