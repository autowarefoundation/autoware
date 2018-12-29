#ifndef SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED
#define SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED

//headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/SystemStatus.h>

//headers in ROS
#include <ros/ros.h>

//headers in STL
#include <mutex>

namespace autoware_health_checker
{
    class SystemStatusSubscriber
    {
    public:
        SystemStatusSubscriber(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~SystemStatusSubscriber();
        void ENABLE();
    private:
        void systemStatusCallback(const autoware_system_msgs::SystemStatus::ConstPtr msg);
        std::mutex mtx_;
    };
}

#endif  //SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED