#include "control_cmd_monitor.h"

namespace autoware_rviz_plugins{
    ControlCommandMonitor::ControlCommandMonitor() : rviz::Display(){
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
        topic_property_ = boost::make_shared<rviz::RosTopicProperty>("Topic", "",ros::message_traits::datatype<autoware_msgs::ControlCommandStamped>(),"autoware_msgs::ControlCommandStamped topic to subscribe to.",this, SLOT(update_topic_()));
    }

    ControlCommandMonitor::~ControlCommandMonitor(){

    }

    void ControlCommandMonitor::onInitialize(){
        update_top_();
        update_left_();
        update_alpha_();
        update_speed_unit_();
        update_topic_();
        return;
    }

    void ControlCommandMonitor::reset(){
        return;
    }

    void ControlCommandMonitor::update(float wall_dt, float ros_dt){
        draw_monitor_();
        return;
    }

    void ControlCommandMonitor::update_speed_unit_(){
        boost::mutex::scoped_lock lock(mutex_);
        speed_unit_ = speed_unit_property_->getOptionInt();
        return;
    }

    void ControlCommandMonitor::update_angle_unit_(){
        boost::mutex::scoped_lock lock(mutex_);
        angle_unit_ = angle_unit_property_->getOptionInt();
        return;
    }

    void ControlCommandMonitor::draw_monitor_(){
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
            ss << "ControlCommandMonitorObject" << count++;
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
        // draw handle circle
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(QColor(0,255,255,(int)(255*alpha_)).rgba()));
        painter.rotate(0);
        QRect handle_rect(20.0, 40.0, 80.0, 80.0);
        painter.drawEllipse(handle_rect);
        // draw handle center
        QPointF handle_center = QPointF(60,80);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setPen(QPen(QColor(0,255,255,(int)(255*alpha_)).rgba()));
        painter.translate(handle_center);
        painter.rotate(-1*last_command_data_.get().cmd.steering_angle*180/M_PI);
        QPointF points[4] = {QPointF(-20.0,-5.0),QPointF(20.0,-5.0),QPointF(10.0,15.0),QPointF(-10.0,15.0)};
        painter.drawConvexPolygon(points, 4);
        painter.rotate(last_command_data_.get().cmd.steering_angle*180/M_PI);
        painter.translate(-handle_center);
        // draw speed meter
        if(speed_unit_==KM_PER_HOUR){
            painter.drawText(QPointF(120,75),QString(("Speed : " + std::to_string(last_command_data_.get().cmd.linear_velocity*3.6) + " km/h").c_str()));
        }
        if(speed_unit_==M_PER_SEC){
            painter.drawText(QPointF(120,75),QString(("Speed : " + std::to_string(last_command_data_.get().cmd.linear_velocity) + " m/s").c_str()));
        }
        // draw steering angle
        if(angle_unit_==RAD){
            painter.drawText(QPointF(120,95),QString(("Steering : " + std::to_string(last_command_data_.get().cmd.steering_angle) + " radian").c_str()));
        }
        if(angle_unit_==DEG){
            painter.drawText(QPointF(120,95),QString(("Steering : " + std::to_string(last_command_data_.get().cmd.steering_angle*180/M_PI) + " degree").c_str()));
        }
        // draw topic name
        painter.drawText(QPointF(100,20),QString(("topic :" + topic_name_).c_str()));
        // draw gear shift position
        painter.drawText(QPointF(10,160),QString("Gear : "));
        painter.setPen(QPen(QColor(150,150,150,(int)(255*alpha_)).rgba()));
        painter.drawText(QPointF(60,160),QString("P"));
        painter.drawText(QPointF(75,160),QString("R"));
        painter.drawText(QPointF(90,160),QString("N"));
        painter.drawText(QPointF(105,160),QString("B"));
        painter.drawText(QPointF(120,160),QString("D"));
        // draw mode function
        painter.setPen(QPen(QColor(0,255,255,(int)(255*alpha_)).rgba()));
        painter.drawText(QPointF(10,180),QString("Drivemode : "));
        painter.setPen(QPen(QColor(150,150,150,(int)(255*alpha_)).rgba()));
        painter.drawText(QPointF(100,180),QString("MANUAL"));
        painter.drawText(QPointF(170,180),QString("AUTO"));
        // draw brake pedal function
        painter.setPen(QPen(QColor(0,255,255,(int)(255*alpha_)).rgba()));
        painter.drawText(QPointF(60,310),QString("Brake Pedal"));
        painter.setPen(QPen(QColor(150,150,150,(int)(255*alpha_)).rgba()));
        QPointF brake_pedal_points[4] = {QPointF(80,290),QPointF(80.0,190.0),QPointF(120.0,190.0),QPointF(120.0,290.0)};
        painter.drawConvexPolygon(brake_pedal_points, 4);
        // draw drive pedal function
        painter.setPen(QPen(QColor(0,255,255,(int)(255*alpha_)).rgba()));
        painter.drawText(QPointF(180,310),QString("Drive Pedal"));
        painter.setPen(QPen(QColor(150,150,150,(int)(255*alpha_)).rgba()));
        QPointF drive_pedal_points[4] = {QPointF(200.0,290.0),QPointF(200.0,190.0),QPointF(240.0,190.0),QPointF(240.0,290.0)};
        painter.drawConvexPolygon(drive_pedal_points, 4);
        return;
    }

    void ControlCommandMonitor::processMessage(const autoware_msgs::ControlCommandStamped::ConstPtr& msg){
        boost::mutex::scoped_lock lock(mutex_);
        last_command_data_ = *msg;
        return;
    }

    void ControlCommandMonitor::update_topic_(){
        boost::mutex::scoped_lock lock(mutex_);
        sub_.shutdown();
        last_command_data_ = boost::none;
        topic_name_ = topic_property_->getTopicStd();
        if (topic_name_.length() > 0 && topic_name_ != "/")
        {
            sub_ = nh_.subscribe(topic_name_, 1, &ControlCommandMonitor::processMessage, this);
        }
        return;
    }

    void  ControlCommandMonitor::update_top_(){
        boost::mutex::scoped_lock lock(mutex_);
        monitor_top_ = top_property_->getInt();
        return;
    }

    void  ControlCommandMonitor::update_left_(){
        boost::mutex::scoped_lock lock(mutex_);
        monitor_left_ = left_property_->getInt();
        return;
    }

    void  ControlCommandMonitor::update_alpha_(){
        boost::mutex::scoped_lock lock(mutex_);
        alpha_ = alpha_property_->getFloat();
        return;
    }

    void ControlCommandMonitor::onEnable(){
        if (overlay_) {
            overlay_->show();
        }
        //subscribe();
        return;
    }

    void ControlCommandMonitor::onDisable(){
        if (overlay_) {
            overlay_->hide();
        }
        //unsubscribe();
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::ControlCommandMonitor, rviz::Display)