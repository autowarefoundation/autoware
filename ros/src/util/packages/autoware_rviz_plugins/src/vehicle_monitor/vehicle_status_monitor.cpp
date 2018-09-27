#include "vehicle_status_monitor.h"

namespace autoware_rviz_plugins{
    VehicleStatusMonitor::VehicleStatusMonitor() : rviz::Display(){
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
        status_topic_property_ = boost::make_shared<rviz::RosTopicProperty>("VehicleStatus Topic", "",ros::message_traits::datatype<autoware_msgs::VehicleStatus>(),"autoware_msgs::VehicleStatus topic to subscribe to.",this, SLOT(update_status_topic_()));
        ctrl_mode_topic_property_ = boost::make_shared<rviz::RosTopicProperty>("CtrlCmd Topic", "",ros::message_traits::datatype<std_msgs::String>(),"std_msgs::String topic to subscribe to.",this, SLOT(update_ctrl_mode_topic__()));
    }

    VehicleStatusMonitor::~VehicleStatusMonitor(){

    }

    void VehicleStatusMonitor::onInitialize(){
        overlay_ = boost::make_shared<OverlayObject>("VehicleStatusMonitor");
        update_top_();
        update_left_();
        update_alpha_();
        update_speed_unit_();
        update_width_();
        update_font_size_();
        update_status_topic_();
        return;
    }

    void VehicleStatusMonitor::reset(){
        return;
    }

    void VehicleStatusMonitor::update(float wall_dt, float ros_dt){
        draw_monitor_();
        return;
    }

    void VehicleStatusMonitor::update_font_size_(){
        boost::mutex::scoped_lock lock(mutex_);
        font_size_ = font_size_property_->getInt();
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

    void VehicleStatusMonitor::update_width_()
    {
        boost::mutex::scoped_lock lock(mutex_);
        width_ = width_property_->getInt();
        height_ = width_;
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
        double width_ratio = (double)width_ / (double)DEFAULT_MONITOR_WIDTH;
        double height_ratio = (double)height_ / (double)DEFAULT_MONITOR_WIDTH;
        /*
        Setup QPainter
        */
        QPainter painter(&Hud);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(QColor(0,255,255,(int)(255*alpha_)).rgba()));
        QFont font;
        font.setPixelSize(font_size_);
        painter.setFont(font);
        /*
        draw gear shift
        */
        QPointF gear_display_position = QPointF(30.0 * width_ratio, 20.0 * height_ratio);
        painter.translate(gear_display_position);
        if(last_command_data_.get().gearshift == GEAR_DRIVE)
            painter.drawText(QPointF(0,0),QString("DRIVE"));
        else if(last_command_data_.get().gearshift == GEAR_REAR)
            painter.drawText(QPointF(0,0),QString("REAR"));
        else if(last_command_data_.get().gearshift == GEAR_BREAK)
            painter.drawText(QPointF(0,0),QString("BREAK"));
        else if(last_command_data_.get().gearshift == GEAR_NEUTRAL)
            painter.drawText(QPointF(0,0),QString("NEUTRAL"));
        else if(last_command_data_.get().gearshift == GEAR_PARKING)
            painter.drawText(QPointF(0,0),QString("PARKING"));
        else
            painter.drawText(QPointF(0,0),QString("UNDEFINED"));
        painter.translate(gear_display_position);
        /*
        draw left lamp
        */
        /*
        QPointF left_lamp_display_position = QPointF(15 * width_ / DEFAULT_MONITOR_WIDTH, 20 * height_ / DEFAULT_MONITOR_WIDTH);
        painter.translate(left_lamp_display_position);
        QPointF lamp_left_points[3] = {QPointF(0.0,0.0),QPointF(0.0,0.0),QPointF(0.0,0.0)};
        painter.translate(-left_lamp_display_position);
        */
        return;
    }

    void VehicleStatusMonitor::processMessage(const autoware_msgs::VehicleStatus::ConstPtr& msg){
        boost::mutex::scoped_lock lock(mutex_);
        last_command_data_ = *msg;
        return;
    }

    void VehicleStatusMonitor::update_ctrl_mode_topic_(){
        boost::mutex::scoped_lock lock(mutex_);
        ctrl_mode_sub_.shutdown();
        last_command_data_ = boost::none;
        ctrl_mode_topic_name_ = status_topic_property_->getTopicStd();
        if (ctrl_mode_topic_name_.length() > 0 && ctrl_mode_topic_name_ != "/")
        {
            ctrl_mode_sub_ = nh_.subscribe(ctrl_mode_topic_name_, 1, &VehicleStatusMonitor::processMessage, this);
        }
        return;
    }

    void VehicleStatusMonitor::update_status_topic_(){
        boost::mutex::scoped_lock lock(mutex_);
        status_sub_.shutdown();
        last_command_data_ = boost::none;
        topic_name_ = status_topic_property_->getTopicStd();
        if (topic_name_.length() > 0 && topic_name_ != "/")
        {
            status_sub_ = nh_.subscribe(topic_name_, 1, &VehicleStatusMonitor::processMessage, this);
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
        return;
    }

    void VehicleStatusMonitor::onDisable(){
        if (overlay_) {
            overlay_->hide();
        }
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::VehicleStatusMonitor, rviz::Display)