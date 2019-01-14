#ifndef RATE_CHECKER_H_INCLUDED
#define RATE_CHECKER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

//headers in STL
#include <vector>
#include <mutex>

//headers in Boost
#include <boost/optional.hpp>

class rate_checker
{
    public:
        rate_checker(double buffer_length);
        ~rate_checker();
        void check();
        boost::optional<double> get_rate();
    private:
        ros::Time start_time_;
        void update_();
        std::vector<ros::Time> data_;
        const double buffer_length_;
        std::mutex mtx_;
};
#endif  //RATE_CHECKER_H_INCLUDED