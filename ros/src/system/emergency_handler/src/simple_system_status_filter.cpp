#include "simple_system_status_filter.h"

const int DIAG_OK = autoware_system_msgs::DiagnosticStatus::OK;
const int DIAG_WARN = autoware_system_msgs::DiagnosticStatus::WARN;
const int DIAG_ERROR = autoware_system_msgs::DiagnosticStatus::ERROR;
const int DIAG_FATAL = autoware_system_msgs::DiagnosticStatus::FATAL;

const std::map<int, std::string> CommonFilterRule::getBehaviorParam(ros::NodeHandle& pnh)
{
  std::map<std::string, int> emergency_stop_map;
  std::map<int, std::string> behavior_param;
  pnh.getParam("behavior/emergency_stop", emergency_stop_map);
  if (emergency_stop_map.size() == 1)
  {
    const auto& el = emergency_stop_map.begin();
    emergency_stop_ = el->second;
    behavior_param.emplace(el->second, el->first);
  }
  return behavior_param;
}

int CommonFilterRule::emergency_stop_ = 0;

int SimpleHardwareFilter::selectPriority(const SystemStatus& status)
{
  const bool is_emergency = !(checkAllHardwareSimplly(status.hardware_status, DIAG_ERROR));
  return is_emergency ? emergency_stop_ : normal_behavior_;
}

int SimpleNodeFilter::selectPriority(const SystemStatus& status)
{
  const bool is_emergency = !(checkAllNodeSimplly(status.node_status, DIAG_ERROR));
  return is_emergency ? emergency_stop_ : normal_behavior_;
}
