#ifndef __SIMPLE_SYSTEM_STATUS_FILTER_H__
#define __SIMPLE_SYSTEM_STATUS_FILTER_H__

#include <emergency_handler/libsystem_status_filter.h>

class SimpleHardwareFilter : public SystemStatusFilter
{
public:
  virtual std::string selectBehavior(const SystemStatus& status);
};

class SimpleNodeFilter : public SystemStatusFilter
{
public:
  virtual std::string selectBehavior(const SystemStatus& status);
};

#endif
