#ifndef __SIMPLE_SYSTEM_STATUS_FILTER_H__
#define __SIMPLE_SYSTEM_STATUS_FILTER_H__

#include <emergency_handler/libsystem_status_filter.h>

class CommonFilterRule : public SystemStatusFilter
{
public:
  static const std::map<int, std::string> getBehaviorParam(ros::NodeHandle& pnh);
protected:
  static int emergency_stop_;
};

class SimpleHardwareFilter : public CommonFilterRule
{
public:
  virtual int selectPriority(const SystemStatus& status);
};

class SimpleNodeFilter : public CommonFilterRule
{
public:
  virtual int selectPriority(const SystemStatus& status);
};

#endif
