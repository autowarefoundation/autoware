#include "control_cmd_monitor.h"

namespace autoware_rviz_plugins{
    ControlCommandMonitor::ControlCommandMonitor() : rviz::Display(){
        topic_property_ = boost::make_shared<rviz::RosTopicProperty>("Topic", "",ros::message_traits::datatype<autoware_msgs::ControlCommandStamped>(),"autoware_msgs::ControlCommandStamped topic to subscribe to.",this, SLOT(update_topic_()));
        left_property_ = boost::make_shared<rviz::IntProperty>("Left position", 0, "Left position of the monitor.",this, SLOT(update_left_()));
        top_property_ = boost::make_shared<rviz::IntProperty>("Top position", 0, "Top position of the monitor.",this, SLOT(update_top_()));
    }

    ControlCommandMonitor::~ControlCommandMonitor(){

    }

    void ControlCommandMonitor::onInitialize(){
        return;
    }

    void ControlCommandMonitor::reset(){
        return;
    }

    void ControlCommandMonitor::processMessage(const autoware_msgs::ControlCommandStamped::ConstPtr& msg){
        return;
    }

    void ControlCommandMonitor::update_topic_(){
        sub_.shutdown();
        std::string topic_name = topic_property_->getTopicStd();
        if (topic_name.length() > 0 && topic_name != "/")
        {
            sub_ = nh_.subscribe(topic_name, 1, &ControlCommandMonitor::processMessage, this);
        }
        return;
    }

    void  ControlCommandMonitor::update_top_(){

    }

    void  ControlCommandMonitor::update_left_(){

    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::ControlCommandMonitor, rviz::Display)