#ifndef DIAG_BUFFER_H_INCLUDED
#define DIAG_BUFFER_H_INCLUDED

//headers in Autoare
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/DiagnosticStatus.h>

//headers in STL
#include <vector>
#include <string>
#include <map>
#include <mutex>

//headers in ROS
#include <ros/ros.h>

class DiagBuffer
{
public:
    DiagBuffer(std::string key,double buffer_length);
    ~DiagBuffer();
    void addDiag(autoware_system_msgs::DiagnosticStatus status);
    uint8_t getErrorLevel();
private:
    std::mutex mtx_;
    void updateBuffer();
    std::string key_;
    ros::Duration buffer_length_;
    std::map<uint8_t,std::vector<std::pair<ros::Time,autoware_system_msgs::DiagnosticStatus> > > buffer_;
    std::vector<std::pair<ros::Time,autoware_system_msgs::DiagnosticStatus> > filterBuffer(ros::Time now, uint8_t level);
};

#endif  //DIAG_BUFFER_H_INCLUDED