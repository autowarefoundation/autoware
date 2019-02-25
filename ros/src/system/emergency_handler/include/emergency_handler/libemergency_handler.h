#ifndef __EMERGENCY_HANDLER_H__
#define __EMERGENCY_HANDLER_H__
#include <ros/ros.h>
#include <autoware_health_checker/system_status_subscriber.h>
#include <autoware_system_msgs/SystemStatus.h>
#include <emergency_handler/libsystem_status_filter.h>
#include <emergency_handler/libemergency_plan_client.h>

typedef boost::function<int(const SystemStatus&)> FilterFunc;

class EmergencyHandler
{
public:
  EmergencyHandler(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~EmergencyHandler();
  void addPublisher(const std::map<int, std::string>& behavior);
  void addFilter(const SystemStatusFilter& filter);
  void run();

private:
  ros::AsyncSpinner spinner_;
  std::vector<ros::Publisher> pub_;
  std::map<int, boost::shared_ptr<EmergencyPlanClient>> emplan_client_;
  std::mutex level_mutex_;
  int priority_;
  int record_level_thresh_;
  autoware_health_checker::SystemStatusSubscriber status_sub_;
  void wrapFunc(FilterFunc func, SystemStatus status);
  void reserveCallback(SystemStatus status);
};

#endif
