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
    uint8_t ret = autoware_health_checker::LEVEL_UNDEFINED;
    updateBuffer();
    for(auto buffer_itr = buffer_.begin(); buffer_itr != buffer_.end(); buffer_itr++)
    {
    }
    return ret;
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