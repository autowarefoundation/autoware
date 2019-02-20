#ifndef __EMERGENCY_HANDLER_H__
#define __EMERGENCY_HANDLER_H__
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <autoware_health_checker/system_status_subscriber.h>
#include <autoware_system_msgs/SystemStatus.h>
#include <emergency_handler/libsystem_status_filter.h>

class EmergencyHandler
{
public:
  EmergencyHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~EmergencyHandler();
  void addPublisher(const std::map<int, std::string>& behavior);
  void addFilter(const SystemStatusFilter& filter);
  void run();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher statecmd_pub_, recordcmd_pub_;
  std::map<int, ros::Publisher> emergency_pub_;
  int handling_level_;
  int record_level_thresh_;
  autoware_health_checker::SystemStatusSubscriber status_sub_;
  void wrapFunc(std::function<int(const SystemStatus&)> func, SystemStatus status);
  void publishCallback(SystemStatus status);
};

#endif
