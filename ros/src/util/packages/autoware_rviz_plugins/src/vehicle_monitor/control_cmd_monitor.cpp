#include "control_cmd_monitor.h"

namespace autoware_rviz_plugins{
    ControlCommandMonitor::ControlCommandMonitor() : rviz::Display(){
        warn_threshold_ = 3.0;
        error_threshold_ = 5.0;
        left_property_ = boost::make_shared<rviz::IntProperty>("Left position", 0, "Left position of the monitor.",this, SLOT(update_left_()));
        top_property_ = boost::make_shared<rviz::IntProperty>("Top position", 0, "Top position of the monitor.",this, SLOT(update_top_()));
        alpha_property_ = boost::make_shared<rviz::FloatProperty>("Alpha", 0, "Top position of the monitor.",this, SLOT(update_alpha_()));
        topic_property_ = boost::make_shared<rviz::RosTopicProperty>("Topic", "",ros::message_traits::datatype<autoware_msgs::ControlCommandStamped>(),"autoware_msgs::ControlCommandStamped topic to subscribe to.",this, SLOT(update_topic_()));
    }

    ControlCommandMonitor::~ControlCommandMonitor(){

    }

    void ControlCommandMonitor::onInitialize(){
        return;
    }

    void ControlCommandMonitor::reset(){
        return;
    }

    void ControlCommandMonitor::draw_monitor(){
        boost::mutex::scoped_lock lock(mutex_);
        if(!last_command_data_)
            return;
        
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
        std::string topic_name = topic_property_->getTopicStd();
        if (topic_name.length() > 0 && topic_name != "/")
        {
            sub_ = nh_.subscribe(topic_name, 1, &ControlCommandMonitor::processMessage, this);
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
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::ControlCommandMonitor, rviz::Display)