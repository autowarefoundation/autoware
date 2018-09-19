#include "vhicle_state_monitor.h"

namespace autoware_rviz_plugins{
    VhicleStateMonitor::VhicleStateMonitor(){

    }

    VhicleStateMonitor::~VhicleStateMonitor(){

    }

    void VhicleStateMonitor::onInitialize() { MFDClass::onInitialize(); }

    void VhicleStateMonitor::reset() { MFDClass::reset(); }

    void VhicleStateMonitor::processMessage(const autoware_msgs::VehicleStatus::ConstPtr& msg){

    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VhicleStateMonitor, rviz::Display)