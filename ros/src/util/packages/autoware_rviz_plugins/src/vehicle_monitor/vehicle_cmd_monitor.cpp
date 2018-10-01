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

    void VehicleCmdMonitor::update(float wall_dt, float ros_dt){
        return;
    }

    void VehicleCmdMonitor::onEnable(){
        return;
    }

    void VehicleCmdMonitor::onDisable(){
        return;
    }

    void VehicleCmdMonitor::processMessage(const autoware_msgs::VehicleCmd::ConstPtr& msg){
        return;
    }

    void VehicleCmdMonitor::update_ctrl_mode_topic_(){
        return;
    }

    void VehicleCmdMonitor::update_status_topic_(){
        return;
    }

    void VehicleCmdMonitor::update_top_(){
        return;
    }

    void VehicleCmdMonitor::update_left_(){
        return;
    }

    void VehicleCmdMonitor::update_alpha_(){
        return;
    }

    void VehicleCmdMonitor::update_speed_unit_(){
        return;
    }

    void VehicleCmdMonitor::update_angle_unit_(){
        return;
    }

    void VehicleCmdMonitor::update_width_(){
        return;
    }

    void VehicleCmdMonitor::update_font_size_(){
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleCmdMonitor, rviz::Display)