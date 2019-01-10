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

#include "state_machine_core.h"

namespace state_machine
{
// Constructor
StateMachineNode::StateMachineNode() : private_nh_("~"), sc_()
{
  initForROS();
}

// Destructor
StateMachineNode::~StateMachineNode()
{
}

void StateMachineNode::initForROS()
{
  // ros parameter settings
  private_nh_.param<bool>("is_manual_light_detection", is_manual_light_detection_, true);

  // setup subscriber
  sub1_ = nh_.subscribe("light_color", 100, &StateMachineNode::callbackFromLightColor, this);
  sub2_ = nh_.subscribe("light_color_managed", 100, &StateMachineNode::callbackFromLightColorManaged, this);

  sub3_ = nh_.subscribe("change_flag", 100, &StateMachineNode::callbackFromChangeFlag, this);

  // setup publisher
  pub_ = nh_.advertise<std_msgs::String>("state", 10);
}

void StateMachineNode::run()
{
  ros::spin();
}

void StateMachineNode::publish() const
{
  /*
  std_msgs::Int32 msg;
  msg.data = sc_.getCurrentState();
  ROS_INFO("Current State: %d",sc_.getCurrentState());
  pub_.publish(msg);
  */

  std_msgs::String msg;
  msg.data = *sc_.getCurrentStateString();
  ROS_INFO_STREAM("Current State String : " << msg.data);
  pub_.publish(msg);
}

void StateMachineNode::callbackFromLightColor(const autoware_msgs::TrafficLightConstPtr& msg)
{
  ROS_INFO("Light color callback");
  if (is_manual_light_detection_)
    return;

  sc_.setLightColor(msg->traffic_light);
  sc_.update();
  publish();
}

void StateMachineNode::callbackFromLightColorManaged(const autoware_msgs::TrafficLightConstPtr& msg)
{
  ROS_INFO("Light color managed callback");
  if (!is_manual_light_detection_)
    return;

  sc_.setLightColor(msg->traffic_light);
  sc_.update();
  publish();
}

void StateMachineNode::callbackFromChangeFlag(const std_msgs::Int32ConstPtr& msg)
{
  ROS_INFO("Change flag callback: %d",msg->data);
  sc_.setChangeFlag(msg->data);
  sc_.update();
  publish();
}
}  // state_machine
