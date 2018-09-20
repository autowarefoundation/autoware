#include "vehicle_cmd_monitor.h"

namespace autoware_rviz_plugins{
    VehicleCmdMonitor::VehicleCmdMonitor(){

    }

    VehicleCmdMonitor::~VehicleCmdMonitor(){

    }

    void VehicleCmdMonitor::onInitialize() { MFDClass::onInitialize(); }

    void VehicleCmdMonitor::reset() { MFDClass::reset(); }

    void VehicleCmdMonitor::processMessage(const autoware_msgs::VehicleCmd::ConstPtr& msg){

    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleCmdMonitor, rviz::Display)