#ifndef __LIBEMERGENCY_PLANNER_H__
#define __LIBEMERGENCY_PLANNER_H__
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
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <autoware_system_msgs/EmergencyAction.h>

class EmergencyPlanner
{
public:
  EmergencyPlanner(std::string name);
  ~EmergencyPlanner();
  virtual void goalCallback();
  virtual void preemptCallback();
  const bool isActive();
  const bool isNewGoalAvailable();
  void setAborted();
  void setSucceeded();
  void publishFeedback(const autoware_msgs::VehicleCmd& vel);
  void publishFeedback(const autoware_msgs::Lane& lane);
protected:
  ros::NodeHandle nh_;
  int priority_;
  autoware_system_msgs::EmergencyFeedback feedback_;
  autoware_system_msgs::EmergencyResult result_;
private:
  actionlib::SimpleActionServer<autoware_system_msgs::EmergencyAction> action_server_;
  void goalCallbackForPrivate();
  void preemptCallbackForPrivate();
};

#endif
