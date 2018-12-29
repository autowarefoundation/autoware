#include <autoware_health_checker/diag_buffer.h>

DiagBuffer::DiagBuffer(std::string key,double buffer_length)
{
    key_ = key;
    buffer_length_ = ros::Duration(buffer_length_);
}

DiagBuffer::~DiagBuffer()
{
    
}

void DiagBuffer::addDiag(autoware_system_msgs::DiagnosticStatus status)
{
    std::lock_guard<std::mutex> lock(mtx_);
    std::pair<ros::Time,autoware_system_msgs::DiagnosticStatus> data;
    data.first = ros::Time::now();
    data.second = status;
    buffer_[status.level].push_back(data);
    updateBuffer();
    return;
}

uint8_t DiagBuffer::getErrorLevel()
{
    std::lock_guard<std::mutex> lock(mtx_);
    updateBuffer();

    return autoware_health_checker::LEVEL_UNDEFINED;
}

// filter data from timestamp and level
std::vector<std::pair<ros::Time,autoware_system_msgs::DiagnosticStatus> > DiagBuffer::filterBuffer(ros::Time now, uint8_t level)
{
    std::vector<std::pair<ros::Time,autoware_system_msgs::DiagnosticStatus> > filterd_data;
    std::vector<std::pair<ros::Time,autoware_system_msgs::DiagnosticStatus> > ret;
    decltype(buffer_)::iterator it = buffer_.find(level);
    if(it != buffer_.end())
    {
        it->second = filterd_data;
    }
    for(auto data_itr = filterd_data.begin(); data_itr != filterd_data.end(); data_itr++)
    {
        if(data_itr->first > (now - buffer_length_))
        {
            ret.push_back(*data_itr);
        }
    }
    return ret;
}

void DiagBuffer::updateBuffer()
{
    ros::Time now = ros::Time::now();
    buffer_[autoware_health_checker::LEVEL_FATAL] = filterBuffer(now, autoware_health_checker::LEVEL_FATAL);
    buffer_[autoware_health_checker::LEVEL_ERROR] = filterBuffer(now, autoware_health_checker::LEVEL_ERROR);
    buffer_[autoware_health_checker::LEVEL_WARN] = filterBuffer(now, autoware_health_checker::LEVEL_WARN);
    buffer_[autoware_health_checker::LEVEL_OK] = filterBuffer(now, autoware_health_checker::LEVEL_OK);
    buffer_[autoware_health_checker::LEVEL_UNDEFINED] = filterBuffer(now, autoware_health_checker::LEVEL_UNDEFINED);
    return;
}