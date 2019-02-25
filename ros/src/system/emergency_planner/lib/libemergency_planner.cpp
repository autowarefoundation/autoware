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
#include <emergency_planner/libemergency_planner.h>

EmergencyPlanner::EmergencyPlanner(std::string name) :
  action_server_(nh_, name, false)
{
  action_server_.registerGoalCallback(boost::bind(&EmergencyPlanner::goalCallbackForPrivate, this));
  action_server_.registerPreemptCallback(boost::bind(&EmergencyPlanner::preemptCallbackForPrivate, this));
  action_server_.start();
}

EmergencyPlanner::~EmergencyPlanner(){}

void EmergencyPlanner::goalCallback(){}

void EmergencyPlanner::goalCallbackForPrivate()
{
  priority_ = action_server_.acceptNewGoal()->priority;
  goalCallback();
}

void EmergencyPlanner::preemptCallback(){}

void EmergencyPlanner::preemptCallbackForPrivate()
{
  action_server_.setPreempted();
  preemptCallback();
}

const bool EmergencyPlanner::isActive()
{
  return action_server_.isActive();
}

const bool EmergencyPlanner::isNewGoalAvailable()
{
  action_server_.isNewGoalAvailable();
}

void EmergencyPlanner::setAborted()
{
  action_server_.setAborted(result_);
}

void EmergencyPlanner::setSucceeded()
{
  action_server_.setSucceeded(result_);
}

void EmergencyPlanner::publishFeedback(const autoware_msgs::VehicleCmd& vel)
{
  feedback_.handling_type = autoware_system_msgs::EmergencyFeedback::TYPE_CONTROL;
  feedback_.control = vel;
  feedback_.plan = autoware_msgs::Lane();
  action_server_.publishFeedback(feedback_);
}

void EmergencyPlanner::publishFeedback(const autoware_msgs::Lane& lane)
{
  feedback_.handling_type = autoware_system_msgs::EmergencyFeedback::TYPE_PLAN;
  feedback_.control = autoware_msgs::VehicleCmd();
  feedback_.plan = lane;
  action_server_.publishFeedback(feedback_);
}
