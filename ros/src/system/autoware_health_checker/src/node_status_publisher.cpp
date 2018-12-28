#include <autoware_health_checker/node_status_publisher.h>

NodeStatusPublisher::NodeStatusPublisher(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
}

NodeStatusPublisher::~NodeStatusPublisher()
{
    
}

void NodeStatusPublisher::CHECK_MIN_VALUE(std::string key,double value,double warn_value,double error_value,double fatal_value)
{

}

void NodeStatusPublisher::CHECK_MAX_VALUE(std::string key,double value,double warn_value,double error_value,double fatal_value)
{

}

void NodeStatusPublisher::CHECK_RANGE(std::string key,double value,std::pair<double,double> warn_value,std::pair<double,double> error_value,std::pair<double,double> fatal_value)
{

}

void NodeStatusPublisher::CHECK_RATE(std::string key,double warn_rate,double error_rate,double fatal_rate)
{

}