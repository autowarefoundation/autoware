#include <autoware_health_checker/diag_buffer.h>

namespace autoware_health_checker
{
    DiagBuffer::DiagBuffer(std::string key, uint8_t type, std::string description, double buffer_length) : type(type), description(description)
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
        buffer_[status.level].push_back(status);
        updateBuffer();
        return;
    }

    std::vector<autoware_system_msgs::DiagnosticStatus> DiagBuffer::getAndClearData()
    {
        std::vector<autoware_system_msgs::DiagnosticStatus> data;
        data = buffer_[autoware_health_checker::LEVEL_FATAL];
        data.insert(data.end(),buffer_[autoware_health_checker::LEVEL_ERROR].begin(),buffer_[autoware_health_checker::LEVEL_ERROR].end());
        data.insert(data.end(),buffer_[autoware_health_checker::LEVEL_WARN].begin(),buffer_[autoware_health_checker::LEVEL_WARN].end());
        data.insert(data.end(),buffer_[autoware_health_checker::LEVEL_OK].begin(),buffer_[autoware_health_checker::LEVEL_OK].end());
        data.insert(data.end(),buffer_[autoware_health_checker::LEVEL_UNDEFINED].begin(),buffer_[autoware_health_checker::LEVEL_UNDEFINED].end());
        buffer_.clear();
        return data;
    }

    uint8_t DiagBuffer::getErrorLevel()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        updateBuffer();
        if(buffer_[autoware_health_checker::LEVEL_FATAL].size() != 0)
        {
            return autoware_health_checker::LEVEL_FATAL;
        }
        else if(buffer_[autoware_health_checker::LEVEL_ERROR].size() != 0)
        {
            return autoware_health_checker::LEVEL_ERROR;
        }
        else if(buffer_[autoware_health_checker::LEVEL_WARN].size() != 0)
        {
            return autoware_health_checker::LEVEL_WARN;
        }
        else
        {
            return autoware_health_checker::LEVEL_OK;
        }
    }

    // filter data from timestamp and level
    std::vector<autoware_system_msgs::DiagnosticStatus> DiagBuffer::filterBuffer(ros::Time now, uint8_t level)
    {
        std::vector<autoware_system_msgs::DiagnosticStatus> filterd_data;
        std::vector<autoware_system_msgs::DiagnosticStatus> ret;
        decltype(buffer_)::iterator it = buffer_.find(level);
        if(it != buffer_.end())
        {
            it->second = filterd_data;
        }
        for(auto data_itr = filterd_data.begin(); data_itr != filterd_data.end(); data_itr++)
        {
            if(data_itr->header.stamp> (now - buffer_length_))
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
}