#include <autoware_health_checker/node_status_publisher.h>

NodeStatusPublisher::NodeStatusPublisher(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    status_pub_ = pnh_.advertise<autoware_system_msgs::NodeStatus>("node_status",10);
}

NodeStatusPublisher::~NodeStatusPublisher()
{
    
}

bool NodeStatusPublisher::keyExist(std::string key)
{
    decltype(diag_buffers_)::iterator it = diag_buffers_.find(key);
    if(it != diag_buffers_.end())
    {
        return true;
    }
    return false;
}

void NodeStatusPublisher::addNewBuffer(std::string key)
{
    if(!keyExist(key))
    {
        std::shared_ptr<DiagBuffer> buf_ptr = std::make_shared<DiagBuffer>(key, autoware_health_checker::BUFFER_LENGTH);
        diag_buffers_[key] = buf_ptr;
    }
    return;
}

void NodeStatusPublisher::CHECK_MIN_VALUE(std::string key,double value,double warn_value,double error_value,double fatal_value)
{
    addNewBuffer(key);
    return;
}

void NodeStatusPublisher::CHECK_MAX_VALUE(std::string key,double value,double warn_value,double error_value,double fatal_value)
{
    addNewBuffer(key);
    return;
}

void NodeStatusPublisher::CHECK_RANGE(std::string key,double value,std::pair<double,double> warn_value,std::pair<double,double> error_value,std::pair<double,double> fatal_value)
{
    addNewBuffer(key);
    return;
}

void NodeStatusPublisher::CHECK_RATE(std::string key,double warn_rate,double error_rate,double fatal_rate)
{
    addNewBuffer(key);
    return;
}