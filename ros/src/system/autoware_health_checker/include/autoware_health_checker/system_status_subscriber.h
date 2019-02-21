#ifndef SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED
#define SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED

//headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/SystemStatus.h>

//headers in ROS
#include <ros/ros.h>

//headers in STL
#include <mutex>
#include <functional>

namespace autoware_health_checker
{
    class SystemStatusSubscriber
    {
    public:
        SystemStatusSubscriber(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~SystemStatusSubscriber();
        void enable();
        void addCallback(std::function<void(autoware_system_msgs::SystemStatus)> func);
    private:
        void systemStatusCallback(const autoware_system_msgs::SystemStatus::ConstPtr msg);
        std::mutex mtx_;
        ros::Subscriber status_sub_;
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        std::vector<std::function<void(autoware_system_msgs::SystemStatus)> > functions_;
    };
}

#endif  //SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED