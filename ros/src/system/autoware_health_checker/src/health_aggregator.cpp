#include <autoware_health_checker/health_aggregator.h>

HealthAggregator::HealthAggregator(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
}

HealthAggregator::~HealthAggregator()
{

}