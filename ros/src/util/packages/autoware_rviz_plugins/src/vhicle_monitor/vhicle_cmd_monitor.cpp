#include "vhicle_cmd_monitor.h"

namespace autoware_rviz_plugins{
    VhicleCmdMonitor::VhicleCmdMonitor(){

    }

    VhicleCmdMonitor::~VhicleCmdMonitor(){

    }

    void VhicleCmdMonitor::onInitialize() { MFDClass::onInitialize(); }

    void VhicleCmdMonitor::reset() { MFDClass::reset(); }

    void VhicleCmdMonitor::processMessage(const autoware_msgs::VehicleCmd::ConstPtr& msg){

    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VhicleCmdMonitor, rviz::Display)