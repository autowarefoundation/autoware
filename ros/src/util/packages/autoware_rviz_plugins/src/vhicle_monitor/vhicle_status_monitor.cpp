#include "vhicle_status_monitor.h"

namespace autoware_rviz_plugins{
    VhicleStatusMonitor::VhicleStatusMonitor(){

    }

    VhicleStatusMonitor::~VhicleStatusMonitor(){

    }

    void VhicleStatusMonitor::onInitialize() { MFDClass::onInitialize(); }

    void VhicleStatusMonitor::reset() { MFDClass::reset(); }

    void VhicleStatusMonitor::processMessage(const autoware_msgs::VehicleStatus::ConstPtr& msg){

    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VhicleStatusMonitor, rviz::Display)