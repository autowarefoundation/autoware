#include <autoware_health_checker/health_aggregator.h>

HealthAggregator::HealthAggregator(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
}

HealthAggregator::~HealthAggregator()
{

}

void HealthAggregator::run()
{
    system_status_pub_ = nh_.advertise<autoware_system_msgs::SystemStatus>("/system_status",10);
    node_status_sub_ = nh_.subscribe("/node_status",10,&HealthAggregator::nodeStatusCallback,this);
    diagnostic_array_sub_ = nh_.subscribe("/diagnostic_agg",10,&HealthAggregator::diagnosticArrayCallback,this);
    return;
}

void HealthAggregator::publishSystemStatus()
{
    return;
}

void HealthAggregator::nodeStatusCallback(const autoware_system_msgs::NodeStatus::ConstPtr msg)
{
    return;
}

void HealthAggregator::diagnosticArrayCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr msg)
{
    return;
}