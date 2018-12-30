#include <ros/ros.h>

#include <autoware_health_checker/health_aggregator.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "health_aggreator");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    HealthAggregator agg(nh,pnh);
    agg.run();
    ros::spin();
    return 0;
}