#include <autoware_health_checker/rate_checker.h>

RateChecker::RateChecker(double buffer_length,double warn_rate,double error_rate,double fatal_rate,std::string description) 
    : buffer_length_(buffer_length), warn_rate_(warn_rate), error_rate_(error_rate), fatal_rate_(fatal_rate)
{
    start_time_ = ros::Time::now();
}

RateChecker::~RateChecker()
{

}

uint8_t RateChecker::getErrorLevel()
{
    boost::optional<double> rate = getRate();
    if(!rate)
    {
        return autoware_health_checker::LEVEL_ERROR;
    }
    if(rate.get() < fatal_rate_)
    {
        return autoware_health_checker::LEVEL_FATAL;
    }
    if(rate.get() < error_rate_)
    {
        return autoware_health_checker::LEVEL_ERROR;
    }
    if(rate.get() < warn_rate_)
    {
        return autoware_health_checker::LEVEL_WARN;
    }
    return autoware_health_checker::LEVEL_OK;
}

void RateChecker::check()
{
    update();
    mtx_.lock();
    data_.push_back(ros::Time::now());
    mtx_.unlock();
}

void RateChecker::update()
{
    std::vector<ros::Time> buffer;
    for(auto data_itr = data_.begin(); data_itr != data_.end(); data_itr++)
    {
        if(*data_itr > ros::Time::now()-ros::Duration(buffer_length_))
        {
            buffer.push_back(*data_itr);
        }
    }
    mtx_.lock();
    data_ = buffer;
    mtx_.unlock();
    return;
}

boost::optional<double> RateChecker::getRate()
{
    boost::optional<double> rate;
    if(ros::Time::now() - start_time_ < ros::Duration(buffer_length_))
    {
        return boost::none;
    }
    update();
    mtx_.lock();
    rate = data_.size()/buffer_length_;
    mtx_.unlock();
    return rate;
}