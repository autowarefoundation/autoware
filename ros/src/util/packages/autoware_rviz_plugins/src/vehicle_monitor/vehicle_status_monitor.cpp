#include "vehicle_status_monitor.h"

namespace autoware_rviz_plugins{
    VehicleStatusMonitor::VehicleStatusMonitor(){

    }

    VehicleStatusMonitor::~VehicleStatusMonitor(){

    }

    void VehicleStatusMonitor::onInitialize() { MFDClass::onInitialize(); }

    void VehicleStatusMonitor::reset() { MFDClass::reset(); }

    void VehicleStatusMonitor::processMessage(const autoware_msgs::VehicleStatus::ConstPtr& msg){

    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleStatusMonitor, rviz::Display)