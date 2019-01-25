#include <diag_lib/rate_checker.h>

rate_checker::rate_checker(double buffer_length) : buffer_length_(buffer_length)
{
    start_time_ = ros::Time::now();
}

rate_checker::~rate_checker()
{

}

void rate_checker::check()
{
    update_();
    mtx_.lock();
    data_.push_back(ros::Time::now());
    mtx_.unlock();
}

void rate_checker::update_()
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

boost::optional<double> rate_checker::get_rate()
{
    boost::optional<double> rate;
    if(ros::Time::now() - start_time_ < ros::Duration(buffer_length_))
    {
        return boost::none;
    }
    update_();
    mtx_.lock();
    rate = data_.size()/buffer_length_;
    mtx_.unlock();
    return rate;
}