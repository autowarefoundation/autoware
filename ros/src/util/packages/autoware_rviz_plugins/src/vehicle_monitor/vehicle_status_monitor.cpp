#include "vehicle_status_monitor.h"

namespace autoware_rviz_plugins{
    VehicleStatusMonitor::VehicleStatusMonitor() : rviz::Display(){
        width_ = 320;
        height_ = 320;
        left_property_ = boost::make_shared<rviz::IntProperty>("Left position", 0, "Left position of the monitor.",this, SLOT(update_left_()));
        top_property_ = boost::make_shared<rviz::IntProperty>("Top position", 0, "Top position of the monitor.",this, SLOT(update_top_()));
        alpha_property_ = boost::make_shared<rviz::FloatProperty>("Alpha", 0, "alpha of the monitor.",this, SLOT(update_alpha_()));
        speed_unit_property_  = boost::make_shared<rviz::EnumProperty>("Speed Unit", "km/h" , "Unit of the speed",this, SLOT(update_speed_unit_()));
        speed_unit_property_->addOption("km/h", KM_PER_HOUR);
        speed_unit_property_->addOption("m/s", M_PER_SEC);
        angle_unit_property_ = boost::make_shared<rviz::EnumProperty>("Angle Unit", "rad" , "Unit of the angle",this, SLOT(update_angle_unit_()));
        angle_unit_property_->addOption("rad", RAD);
        angle_unit_property_->addOption("deg", DEG);
        topic_property_ = boost::make_shared<rviz::RosTopicProperty>("Topic", "",ros::message_traits::datatype<autoware_msgs::VehicleStatus>(),"autoware_msgs::VehicleStatus topic to subscribe to.",this, SLOT(update_topic_()));
    }

    VehicleStatusMonitor::~VehicleStatusMonitor(){

    }

    void VehicleStatusMonitor::onInitialize(){
        update_top_();
        update_left_();
        update_alpha_();
        update_speed_unit_();
        update_topic_();
        return;
    }

    void VehicleStatusMonitor::reset(){
        return;
    }

    void VehicleStatusMonitor::update(float wall_dt, float ros_dt){
        draw_monitor_();
        return;
    }

    void VehicleStatusMonitor::update_speed_unit_(){
        boost::mutex::scoped_lock lock(mutex_);
        speed_unit_ = speed_unit_property_->getOptionInt();
        return;
    }

    void VehicleStatusMonitor::update_angle_unit_(){
        boost::mutex::scoped_lock lock(mutex_);
        angle_unit_ = angle_unit_property_->getOptionInt();
        return;
    }

    void VehicleStatusMonitor::draw_monitor_(){
        boost::mutex::scoped_lock lock(mutex_);
        if(!last_command_data_)
        {
            return;
        }
        /*
        Functions to draw monitor
        */
        if(!overlay_)
        {
            static int count = 0;
            rviz::UniformStringStream ss;
            ss << "VehicleStatusMonitorObject" << count++;
            overlay_.reset(new OverlayObject(ss.str()));
            overlay_->show();
        }
        if(overlay_)
        {
            overlay_->setDimensions(width_,height_);
            overlay_->setPosition(monitor_left_,monitor_top_);
        }
        overlay_->updateTextureSize(width_,height_);
        ScopedPixelBuffer buffer = overlay_->getBuffer();
        QImage Hud = buffer.getQImage(*overlay_);
        for (unsigned int i = 0; i < overlay_->getTextureWidth(); i++) {
            for (unsigned int j = 0; j < overlay_->getTextureHeight(); j++) {
                Hud.setPixel(i, j, QColor(0,0,0,(int)(255*alpha_)).rgba());
            }
        } 
        QPainter painter(&Hud);
        return;
    }

    void VehicleStatusMonitor::processMessage(const autoware_msgs::VehicleStatus::ConstPtr& msg){
        boost::mutex::scoped_lock lock(mutex_);
        last_command_data_ = *msg;
        return;
    }

    void VehicleStatusMonitor::update_topic_(){
        boost::mutex::scoped_lock lock(mutex_);
        sub_.shutdown();
        last_command_data_ = boost::none;
        topic_name_ = topic_property_->getTopicStd();
        if (topic_name_.length() > 0 && topic_name_ != "/")
        {
            sub_ = nh_.subscribe(topic_name_, 1, &VehicleStatusMonitor::processMessage, this);
        }
        return;
    }

    void  VehicleStatusMonitor::update_top_(){
        boost::mutex::scoped_lock lock(mutex_);
        monitor_top_ = top_property_->getInt();
        return;
    }

    void  VehicleStatusMonitor::update_left_(){
        boost::mutex::scoped_lock lock(mutex_);
        monitor_left_ = left_property_->getInt();
        return;
    }

    void  VehicleStatusMonitor::update_alpha_(){
        boost::mutex::scoped_lock lock(mutex_);
        alpha_ = alpha_property_->getFloat();
        return;
    }

    void VehicleStatusMonitor::onEnable(){
        if (overlay_) {
            overlay_->show();
        }
        //subscribe();
        return;
    }

    void VehicleStatusMonitor::onDisable(){
        if (overlay_) {
            overlay_->hide();
        }
        //unsubscribe();
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleStatusMonitor, rviz::Display)