#include <autoware_health_checker/system_status_subscriber.h>

namespace autoware_health_checker
{
    SystemStatusSubscriber::SystemStatusSubscriber(ros::NodeHandle nh,ros::NodeHandle pnh)
    {
        nh_ = nh;
        pnh_ = pnh;
    }

    SystemStatusSubscriber::~SystemStatusSubscriber()
    {

    }

    void SystemStatusSubscriber::enable()
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::Rate rate(1);
        status_sub_ = nh_.subscribe("system_status",10,&SystemStatusSubscriber::systemStatusCallback,this);
        while(ros::ok())
        {
            rate.sleep();
        }
        spinner.stop();
        return;
    }

    void SystemStatusSubscriber::systemStatusCallback(const autoware_system_msgs::SystemStatus::ConstPtr msg)
    {
        for(auto function_itr = functions_.begin(); function_itr != functions_.end(); function_itr++)
        {
            std::function<void(autoware_system_msgs::SystemStatus)> func = *function_itr;
            func(*msg);
        }
        return;
    }

    void SystemStatusSubscriber::addCallback(std::function<void(autoware_system_msgs::SystemStatus)> func)
    {
        functions_.push_back(func);
        return;
    }
}