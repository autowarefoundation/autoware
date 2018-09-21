#include "vehicle_status_monitor.h"

namespace autoware_rviz_plugins{
    VehicleStatusMonitor::VehicleStatusMonitor() : rviz::Display(){

    }

    VehicleStatusMonitor::~VehicleStatusMonitor(){

    }

    void VehicleStatusMonitor::onInitialize(){
        return;
    }

    void VehicleStatusMonitor::reset(){
        return;
    }

    void VehicleStatusMonitor::processMessage(const autoware_msgs::VehicleStatus::ConstPtr& msg){
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleStatusMonitor, rviz::Display)