#include <emergency_handler/libemergency_handler.h>
#include "simple_system_status_filter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "emergency_handler");
  ros::NodeHandle nh, pnh("~");
  SimpleNodeFilter node_filter;
  SimpleHardwareFilter hw_filter;
  EmergencyHandler emergency_handler(nh, pnh);
  emergency_handler.addFilter((SystemStatusFilter)node_filter);
  emergency_handler.addFilter((SystemStatusFilter)hw_filter);
  emergency_handler.addPublishCallback();
  emergency_handler.enable();
  return 0;
}
