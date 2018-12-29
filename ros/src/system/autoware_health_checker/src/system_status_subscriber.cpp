#include <autoware_health_checker/system_status_subscriber.h>

namespace autoware_health_checker
{
    SystemStatusSubscriber::SystemStatusSubscriber(ros::NodeHandle nh,ros::NodeHandle pnh)
    {

    }

    SystemStatusSubscriber::~SystemStatusSubscriber()
    {

    }

    void SystemStatusSubscriber::ENABLE()
    {
        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::Rate rate(1);
        while(ros::ok())
        {
            rate.sleep();
        }
        spinner.stop();
        return;
    }

    void SystemStatusSubscriber::systemStatusCallback(const autoware_system_msgs::SystemStatus::ConstPtr msg)
    {
        return;
    }
}