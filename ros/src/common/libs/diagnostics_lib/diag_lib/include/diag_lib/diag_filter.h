#ifndef DIAG_FILTER_H_INCLUDED
#define DIAG_FILTER_H_INCLUDED

//headers in diag_msgs
#include <diag_msgs/diag.h>
#include <diag_msgs/diag_error.h>
#include <diag_msgs/diag_node_errors.h>

//headers in ROS
#include <ros/ros.h>

//headers in Boost
#include <boost/optional.hpp>

//headers in STL
#include <map>

class diag_filter
{
public:
    diag_filter();
    ~diag_filter();
    boost::optional<diag_msgs::diag_node_errors> filter(diag_msgs::diag diag, std::string target_node);
    boost::optional<diag_msgs::diag_node_errors> filter(diag_msgs::diag diag, int target_node_number);
private:
    std::map<std::string,int> node_number_data_;
    ros::NodeHandle nh_;
    bool check_resource_(std::string target_resource_path);
    volatile bool enable_;
    std::string error_code_config_path_;
};
#endif  //DIAG_FILTER_H_INCLUDED