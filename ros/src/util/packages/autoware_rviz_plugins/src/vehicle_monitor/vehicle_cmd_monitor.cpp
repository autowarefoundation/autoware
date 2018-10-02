#include "vehicle_cmd_monitor.h"

namespace autoware_rviz_plugins{
    VehicleCmdMonitor::VehicleCmdMonitor() : rviz::Display(){
        gear_status_.load_params();
        control_mode_ = "";
        max_accel_value_property_ = boost::make_shared<rviz::IntProperty>("Max accel value", 4095, "Maximum accel value.",this, SLOT(update_max_accel_value_()));
        min_accel_value_property_ = boost::make_shared<rviz::IntProperty>("Min accel value",    0, "Minimum accel value.",this, SLOT(update_min_accel_value_()));
        max_brake_value_property_ = boost::make_shared<rviz::IntProperty>("Max brake value", 4095, "Maximum brake value.",this, SLOT(update_max_brake_value_()));
        min_brake_value_property_ = boost::make_shared<rviz::IntProperty>("Min brake value",    0, "Minimum brake value.",this, SLOT(update_min_brake_value_()));
        width_property_ = boost::make_shared<rviz::IntProperty>("Monitor width", DEFAULT_MONITOR_WIDTH, "Width of the monitor.",this, SLOT(update_width_()));
        left_property_ = boost::make_shared<rviz::IntProperty>("Left position", 0, "Left position of the monitor.",this, SLOT(update_left_()));
        top_property_ = boost::make_shared<rviz::IntProperty>("Top position", 0, "Top position of the monitor.",this, SLOT(update_top_()));
        font_size_property_ = boost::make_shared<rviz::IntProperty>("Font size", 20, "Top position of the monitor.",this, SLOT(update_font_size_()));
        alpha_property_ = boost::make_shared<rviz::FloatProperty>("Alpha", 0, "alpha of the monitor.",this, SLOT(update_alpha_()));
        speed_unit_property_  = boost::make_shared<rviz::EnumProperty>("Speed unit", "km/h" , "Unit of the speed",this, SLOT(update_speed_unit_()));
        speed_unit_property_->addOption("km/h", KM_PER_HOUR);
        speed_unit_property_->addOption("m/s", M_PER_SEC);
        angle_unit_property_ = boost::make_shared<rviz::EnumProperty>("Angle unit", "rad" , "Unit of the angle",this, SLOT(update_angle_unit_()));
        angle_unit_property_->addOption("rad", RAD);
        angle_unit_property_->addOption("deg", DEG);
        status_topic_property_ = boost::make_shared<rviz::RosTopicProperty>("VehicleCmd Topic", "",ros::message_traits::datatype<autoware_msgs::VehicleCmd>(),"autoware_msgs::VehicleCmd topic to subscribe to.",this, SLOT(update_status_topic_()));
        ctrl_mode_topic_property_ = boost::make_shared<rviz::RosTopicProperty>("CtrlCmd Topic", "",ros::message_traits::datatype<std_msgs::String>(),"std_msgs::String topic to subscribe to.",this, SLOT(update_ctrl_mode_topic_()));
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

    void VehicleCmdMonitor::update_max_accel_value_(){
        return;
    }

    void VehicleCmdMonitor::update_min_accel_value_(){
        return;
    }

    void VehicleCmdMonitor::update_max_brake_value_(){
        return;
    }

    void VehicleCmdMonitor::update_min_brake_value_(){
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleCmdMonitor, rviz::Display)