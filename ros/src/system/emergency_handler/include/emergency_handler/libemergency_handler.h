#ifndef __EMERGENCY_HANDLER_H__
#define __EMERGENCY_HANDLER_H__
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <autoware_health_checker/system_status_subscriber.h>
#include <autoware_system_msgs/SystemStatus.h>
#include <emergency_handler/libsystem_status_filter.h>

class EmergencyHandler : public autoware_health_checker::SystemStatusSubscriber
{
public:
  EmergencyHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~EmergencyHandler();
  void addFilter(const SystemStatusFilter& filter);
  void addPublishCallback();

private:
  ros::Publisher emergency_pub_, statecmd_pub_, recordcmd_pub_;
  bool is_emergency_, start_record_;
  std::string statecmd_str_;
  void changePublisherFlag(std::string behavior);
  void wrapFunc(std::function<std::string(const SystemStatus&)> func, SystemStatus status);
  void publishCallback(SystemStatus status);
};

#endif
