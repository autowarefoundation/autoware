#ifndef HEALTH_AGGREGATOR_H_INCLUDED
#define HEALTH_AGGREGATOR_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

class HealthAggregator
{
public:
    HealthAggregator(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~HealthAggregator();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
};
#endif  //HEALTH_AGGREGATOR_H_INCLUDED