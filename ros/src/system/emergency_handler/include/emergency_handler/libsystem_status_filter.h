#ifndef __SYSTEM_STATUS_FILTER_H__
#define __SYSTEM_STATUS_FILTER_H__
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
  virtual std::string selectBehavior(const SystemStatus& status);
  const std::function<std::string(const SystemStatus&)>& getFunc() const;

protected:
  std::function<std::string(const SystemStatus&)> callback_;

  StatusType getStatus(const DiagnosticStatusArray& st_array, int level_th) const;
  StatusType getStatus(const NodeStatus& node_status, int level_th) const;
  StatusType getStatus(const HardwareStatus& hw_status, int level_th) const;
  StatusType getStatus(std::string node_name, const std::vector<NodeStatus>& array, int level_th) const;
  bool checkAllNodeSimplly(const std::vector<NodeStatus>& array, int level_th) const;
  bool checkAllHardwareSimplly(const std::vector<HardwareStatus>& array, int level_th) const;
  template<typename T> bool checkAllSimplly(const std::vector<T>& array, int level_th) const;
};


#endif
