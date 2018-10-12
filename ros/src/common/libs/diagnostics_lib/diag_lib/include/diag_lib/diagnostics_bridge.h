#ifndef DIAGNOSTIC_BRIDGE_H_INCLUDED
#define DIAGNOSTIC_BRIDGE_H_INCLUDED

//headers in diag_lib
#include <diag_msgs/diag.h>

//headers in ROS
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

//headers in STL
#include <map>

class diagnostics_bridge
{
public:
    diagnostics_bridge();
    ~diagnostics_bridge();
private:
    void diag_callback_(const diag_msgs::diag::ConstPtr msg);
    ros::NodeHandle nh_;
    ros::Subscriber diag_sub_;
    std::map<std::string,diagnostic_updater::Updater> updaters_;
};
#endif  //DIAGNOSTIC_BRIDGE_H_INCLUDED