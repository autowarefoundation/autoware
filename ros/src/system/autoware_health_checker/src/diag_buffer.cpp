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
    buffer_.push_back(data);
    updateBuffer();
    return;
}

uint8_t DiagBuffer::getErrorLevel()
{
    std::lock_guard<std::mutex> lock(mtx_);
    updateBuffer();
    for(auto buffer_itr = buffer_.begin(); buffer_itr != buffer_.end(); buffer_itr++)
    {
        if(buffer_itr->second.type == autoware_health_checker::LEVEL_FATAL)
        {
            return autoware_health_checker::LEVEL_FATAL;
        }
    }
    for(auto buffer_itr = buffer_.begin(); buffer_itr != buffer_.end(); buffer_itr++)
    {
        if(buffer_itr->second.type == autoware_health_checker::LEVEL_ERROR)
        {
            return autoware_health_checker::LEVEL_ERROR;
        }
    }
    for(auto buffer_itr = buffer_.begin(); buffer_itr != buffer_.end(); buffer_itr++)
    {
        if(buffer_itr->second.type == autoware_health_checker::LEVEL_WARN)
        {
            return autoware_health_checker::LEVEL_WARN;
        }
    }
    for(auto buffer_itr = buffer_.begin(); buffer_itr != buffer_.end(); buffer_itr++)
    {
        if(buffer_itr->second.type == autoware_health_checker::LEVEL_OK)
        {
            return autoware_health_checker::LEVEL_OK;
        }
    }
    return autoware_health_checker::LEVEL_UNDEFINED;
}

void DiagBuffer::updateBuffer()
{
    //std::lock_guard<std::mutex> lock(mtx_);
    std::vector<std::pair<ros::Time,autoware_system_msgs::DiagnosticStatus> > new_buffer_;
    ros::Time now = ros::Time::now();
    for(auto buffer_itr = buffer_.begin(); buffer_itr != buffer_.end(); buffer_itr++)
    {
        if(buffer_itr->first > (now - buffer_length_))
        {
            new_buffer_.push_back(*buffer_itr);
        }
    }
    buffer_ = new_buffer_;
    return;
}