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

class EmergencyStopPlanner : public EmergencyPlanner
{
public:
  EmergencyStopPlanner(std::string name) : EmergencyPlanner(name),
    new_order_(false)
  {
    ros::NodeHandle pnh("~");
    pnh.param<double>("stop_signal_secs", stop_signal_secs_, 10.0);
  }
  virtual void goalCallback()
  {
    new_order_ = true;
  }
  void run()
  {
    ros::Rate r(50);
    while (ros::ok())
    {
      if (new_order_)
      {
        new_order_ = false;
        runPlannerServer();
      }
      ros::spinOnce();
      r.sleep();
    }
  }
  void runPlannerServer()
  {
    ros::Time start_time = ros::Time::now();
    ros::Rate r(50);
    while (ros::ok() && !new_order_)
    {
      const ros::Duration diff = ros::Time::now() - start_time;
      if (diff.toSec() > stop_signal_secs_)
      {
        break;
      }
      autoware_msgs::VehicleCmd vehicle_cmd;
      vehicle_cmd.header.stamp = ros::Time::now();
      vehicle_cmd.emergency = 1;
      publishFeedback(vehicle_cmd);
      ros::spinOnce();
      r.sleep();
    }
    new_order_ ? setAborted() : setSucceeded();
  }
private:
  double stop_signal_secs_;
  bool new_order_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emergency_stop_planner");
  EmergencyStopPlanner esp(ros::this_node::getName());
  esp.run();
  return 0;
}
