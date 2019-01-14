#ifndef DIAG_SUBSCRIBER_H_INCLUDED
#define DIAG_SUBSCRIBER_H_INCLUDED

//headers in STL
#include <vector>
#include <mutex>
#include <algorithm>

//headers in ROS
#include <ros/ros.h>

//headers in diag_msgs
#include <diag_msgs/diag_node_errors.h>
#include <diag_msgs/diag_error.h>

class diag_subscriber
{
    public:
        diag_subscriber(std::string target_node,int target_node_number);
        ~diag_subscriber();
        diag_msgs::diag_node_errors get_diag_node_errors();
    private:
        std::mutex mtx_;
        std::vector<diag_msgs::diag_error> buffer_;
        ros::Subscriber diag_sub_;
        ros::NodeHandle nh_;
        void callback_(diag_msgs::diag_error msg);
        const std::string target_node_;
        const int target_node_number_;
};

#endif  //DIAG_SUBSCRIBER_H_INCLUDED