#include "vehicle_cmd_monitor.h"

namespace autoware_rviz_plugins{
    VehicleCmdMonitor::VehicleCmdMonitor() : rviz::Display(){

    }

    VehicleCmdMonitor::~VehicleCmdMonitor(){

    }

    void VehicleCmdMonitor::onInitialize(){
        return;
    }

    void VehicleCmdMonitor::reset(){
        return;
    }

    void VehicleCmdMonitor::processMessage(const autoware_msgs::VehicleCmd::ConstPtr& msg){
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleCmdMonitor, rviz::Display)