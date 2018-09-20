#ifndef FAKE_SUBSCRIBER_H_INCLUDED
#define FAKE_SUBSCRIBER_H_INCLUDED

//headers in diag_lib
#include <diag_lib/diag_manager.h>
#include <diag_lib/diag_filter.h>

//hdaders in diag_msg
#include <diag_msgs/diag_error.h>

//headers in std_msgs
#include <std_msgs/Float64.h>

//headers in ROS
#include <ros/ros.h>

class fake_subscriber
{
    public:
        fake_subscriber();
        ~fake_subscriber();
    private:
        void callback_(const std_msgs::Float64ConstPtr msg);
        ros::NodeHandle nh_;
        ros::Subscriber fake_sub_;
        diag_manager diag_manager_;
        diag_filter diag_filter_;
};

#endif  //FAKE_SUBSCRIBER_H_INCLUDED