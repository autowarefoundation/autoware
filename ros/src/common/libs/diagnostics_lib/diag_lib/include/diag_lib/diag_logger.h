#ifndef DIAG_LOGGER_H_INCLUDED
#define DIAG_LOGGER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <jsk_rviz_plugins/OverlayText.h>

//headers in diag_lib
#include <diag_lib/diag_filter.h>
#include <diag_lib/diag_info.h>

//headers in diag_msgs
#include <diag_msgs/diag.h>

//headers in boost
#include <boost/circular_buffer.hpp>

class diag_logger
{
public:
    diag_logger();
    ~diag_logger();
    void update(diag_msgs::diag data);
private:
    boost::circular_buffer<diag_msgs::diag> buf_;
    int buffer_length_;
    diag_filter filter_;
    jsk_rviz_plugins::OverlayText text_;
};
#endif //DIAG_LOGGER_H_INCLUDED