/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef STATE_MACHINE_CORE_H
#define STATE_MACHINE_CORE_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

// User Defined includes
#include "autoware_msgs/TrafficLight.h"
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
  ros::Subscriber sub1_, sub2_, sub3_;

  // variables
  bool is_manual_light_detection_;

  // callbacks
  void callbackFromLightColor(const autoware_msgs::TrafficLightConstPtr &msg);
  void callbackFromLightColorManaged(const autoware_msgs::TrafficLightConstPtr &msg);
  void callbackFromChangeFlag(const std_msgs::Int32ConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  void publish() const;
};
}  // state_machine
#endif  // STATE_MACHINE_CORE_H
