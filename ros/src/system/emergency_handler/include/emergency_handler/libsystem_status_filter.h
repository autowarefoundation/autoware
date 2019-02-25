#ifndef __SYSTEM_STATUS_FILTER_H__
#define __SYSTEM_STATUS_FILTER_H__
#include <ros/ros.h>
#include <autoware_system_msgs/SystemStatus.h>

typedef autoware_system_msgs::SystemStatus SystemStatus;
typedef autoware_system_msgs::DiagnosticStatusArray DiagnosticStatusArray;
typedef autoware_system_msgs::DiagnosticStatus DiagnosticStatus;
typedef autoware_system_msgs::NodeStatus NodeStatus;
typedef autoware_system_msgs::HardwareStatus HardwareStatus;

enum StatusType
{
  NONE,
  NOT_READY,
  OK,
  ERROR
};

class SystemStatusFilter
{
public:
  SystemStatusFilter();
  virtual int selectPriority(const SystemStatus& status);
  const std::function<int(const SystemStatus&)>& getFunc() const;

protected:
  std::function<int(const SystemStatus&)> callback_;
  static const int normal_behavior_;

  StatusType getStatus(const DiagnosticStatusArray& st_array, int level_th) const;
  StatusType getStatus(const NodeStatus& node_status, int level_th) const;
  StatusType getStatus(const HardwareStatus& hw_status, int level_th) const;
  StatusType getStatus(std::string node_name, const std::vector<NodeStatus>& array, int level_th) const;
  bool checkAllNodeSimplly(const std::vector<NodeStatus>& array, int level_th) const;
  bool checkAllHardwareSimplly(const std::vector<HardwareStatus>& array, int level_th) const;
  template<typename T> bool checkAllSimplly(const std::vector<T>& array, int level_th) const;
};


#endif
