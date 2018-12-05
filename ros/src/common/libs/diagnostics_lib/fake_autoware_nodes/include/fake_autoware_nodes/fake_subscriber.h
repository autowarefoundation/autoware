#ifndef FAKE_SUBSCRIBER_H_INCLUDED
#define FAKE_SUBSCRIBER_H_INCLUDED

//headers in diag_lib
#include <diag_lib/diag_manager.h>
#include <diag_lib/diag_filter.h>

//headers in diag_msg
#include <diag_msgs/diag_error.h>
#include <diag_msgs/diag.h>

//headers in std_msgs
#include <std_msgs/Float64.h>

//headers in ROS
#include <ros/ros.h>

class FakeSubscriber
{
    public:
        FakeSubscriber();
        ~FakeSubscriber();
    private:
        void diag_callback_(const diag_msgs::diagConstPtr msg);
        void callback_(const std_msgs::Float64ConstPtr msg);
        ros::NodeHandle nh_;
        ros::Subscriber fake_sub_;
        ros::Subscriber diag_sub_;
        DiagManager diag_manager_;
        DiagFilter diag_filter_;
};

#endif  //FAKE_SUBSCRIBER_H_INCLUDED