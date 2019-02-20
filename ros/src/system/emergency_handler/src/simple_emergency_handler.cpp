#include <emergency_handler/libemergency_handler.h>
#include "simple_system_status_filter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_emergency_handler");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  SimpleHardwareFilter hw_filter;
  SimpleNodeFilter node_filter;
  EmergencyHandler emergency_handler(nh, pnh);
  emergency_handler.addPublisher(CommonFilterRule::getBehaviorParam(pnh));
  emergency_handler.addFilter((SystemStatusFilter)hw_filter);
  emergency_handler.addFilter((SystemStatusFilter)node_filter);
  emergency_handler.run();
  return 0;
}
