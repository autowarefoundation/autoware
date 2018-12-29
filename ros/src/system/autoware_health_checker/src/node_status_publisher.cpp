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

void NodeStatusPublisher::publishStatus()
{
    ros::Rate rate = ros::Rate(autoware_health_checker::UPDATE_RATE);
    while(ros::ok())
    {
        rate.sleep();
    }
    return;
}

void NodeStatusPublisher::ENABLE()
{
    boost::thread publish_thread(boost::bind(&NodeStatusPublisher::publishStatus, this));
    return;
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

void NodeStatusPublisher::CHECK_MIN_VALUE(std::string key,double value,double warn_value,double error_value,double fatal_value,std::string description)
{
    addNewBuffer(key);
    autoware_system_msgs::DiagnosticStatus new_status;
    new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
    if(value < fatal_value)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
    }
    else if(value < error_value)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
    }
    else if(value < warn_value)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
    }
    else
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
    }
    new_status.description = description;
    new_status.value = doubeToJson(value);
    diag_buffers_[key]->addDiag(new_status);
    return;
}

void NodeStatusPublisher::CHECK_MAX_VALUE(std::string key,double value,double warn_value,double error_value,double fatal_value,std::string description)
{
    addNewBuffer(key);
    autoware_system_msgs::DiagnosticStatus new_status;
    new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
    if(value > fatal_value)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
    }
    else if(value > error_value)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
    }
    else if(value > warn_value)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
    }
    else
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
    }
    new_status.description = description;
    new_status.value = doubeToJson(value);
    diag_buffers_[key]->addDiag(new_status);
    return;
}

void NodeStatusPublisher::CHECK_RANGE(std::string key,double value,std::pair<double,double> warn_value,std::pair<double,double> error_value,std::pair<double,double> fatal_value,std::string description)
{
    addNewBuffer(key);
    autoware_system_msgs::DiagnosticStatus new_status;
    new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
    if(value < fatal_value.first && value > fatal_value.second)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
    }
    else if(value < error_value.first && value > error_value.second)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
    }
    else if(value < warn_value.first && value > warn_value.second)
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
    }
    else
    {
        new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
    }
    new_status.value = doubeToJson(value);
    new_status.description = description;
    diag_buffers_[key]->addDiag(new_status);
    return;
}

void NodeStatusPublisher::CHECK_RATE(std::string key,double warn_rate,double error_rate,double fatal_rate,std::string description)
{
    if(!keyExist(key))
    {
        std::shared_ptr<RateChecker> checker_ptr = std::make_shared<RateChecker>(autoware_health_checker::BUFFER_LENGTH);
        rate_checkers_[key] = checker_ptr;
    }
    addNewBuffer(key);
    rate_checkers_[key]->check();
    return;
}

std::string NodeStatusPublisher::doubeToJson(double value)
{
    using namespace boost::property_tree;
    std::stringstream ss;
    ptree pt;
    pt.put("value.double", value);
    write_json(ss, pt);
    return ss.str();
}