#ifndef __LIBEMERGENCY_PLANNER_H__
#define __LIBEMERGENCY_PLANNER_H__
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
  void setAborted();
  void setSucceeded();
  void publishFeedback(const autoware_msgs::VehicleCmd& vel);
  void publishFeedback(const autoware_msgs::Lane& lane);
protected:
  ros::NodeHandle nh_;
  int goal_;
  autoware_system_msgs::EmergencyFeedback feedback_;
  autoware_system_msgs::EmergencyResult result_;
private:
  actionlib::SimpleActionServer<autoware_system_msgs::EmergencyAction> action_server_;
  void goalCallbackForPrivate();
  void preemptCallbackForPrivate();
};

#endif
