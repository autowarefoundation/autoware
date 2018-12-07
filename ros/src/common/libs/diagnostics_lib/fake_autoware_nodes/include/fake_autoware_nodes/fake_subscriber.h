#ifndef FAKE_SUBSCRIBER_H_INCLUDED
#define FAKE_SUBSCRIBER_H_INCLUDED

// headers in diag_lib
#include <diag_lib/diag_filter.h>
#include <diag_lib/diag_manager.h>

// headers in diag_msg
#include <diag_msgs/diag.h>
#include <diag_msgs/diag_error.h>

// headers in std_msgs
#include <std_msgs/Float64.h>

// headers in ROS
#include <ros/ros.h>

class FakeSubscriber {
public:
  FakeSubscriber();
  ~FakeSubscriber();

private:
  void diagCallback(const diag_msgs::diagConstPtr msg);
  void callback(const std_msgs::Float64ConstPtr msg);
  ros::NodeHandle nh_;
  ros::Subscriber fake_sub_;
  ros::Subscriber diag_sub_;
  DiagManager diag_manager_;
  DiagFilter diag_filter_;
};

#endif // FAKE_SUBSCRIBER_H_INCLUDED