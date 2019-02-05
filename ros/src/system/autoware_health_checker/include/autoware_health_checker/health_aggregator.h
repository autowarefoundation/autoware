#ifndef HEALTH_AGGREGATOR_H_INCLUDED
#define HEALTH_AGGREGATOR_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

//headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/NodeStatus.h>
#include <autoware_system_msgs/SystemStatus.h>

//headers in boost
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/thread.hpp>

//headers in STL
#include <mutex>
#include <map>

class HealthAggregator
{
public:
    HealthAggregator(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~HealthAggregator();
    void run();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher system_status_pub_;
    ros::Subscriber node_status_sub_;
    ros::Subscriber diagnostic_array_sub_;
    void publishSystemStatus();
    void nodeStatusCallback(const autoware_system_msgs::NodeStatus::ConstPtr msg);
    void diagnosticArrayCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr msg);
    boost::optional<autoware_system_msgs::HardwareStatus> convert(const diagnostic_msgs::DiagnosticArray::ConstPtr msg);
    autoware_system_msgs::SystemStatus system_status_;
    std::mutex mtx_;
    void updateConnectionStatus();
};
#endif  //HEALTH_AGGREGATOR_H_INCLUDED