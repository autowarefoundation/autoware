#include "simple_system_status_filter.h"

const int DIAG_OK = autoware_system_msgs::DiagnosticStatus::OK;
const int DIAG_WARN = autoware_system_msgs::DiagnosticStatus::WARN;
const int DIAG_ERROR = autoware_system_msgs::DiagnosticStatus::ERROR;
const int DIAG_FATAL = autoware_system_msgs::DiagnosticStatus::FATAL;

std::string SimpleHardwareFilter::selectBehavior(const SystemStatus& status)
{
  return (checkAllHardwareSimplly(status.hardware_status, DIAG_ERROR)) ? "None" : "Emergency";
}

std::string SimpleNodeFilter::selectBehavior(const SystemStatus& status)
{
  return (checkAllNodeSimplly(status.node_status, DIAG_ERROR)) ? "None" : "Emergency";
}
