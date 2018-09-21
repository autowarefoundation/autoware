#include "control_cmd_monitor.h"

namespace autoware_rviz_plugins{
    ControlCommandMonitor::ControlCommandMonitor() : rviz::Display(){
        update_topic_property_ = new rviz::RosTopicProperty("Topic", "",ros::message_traits::datatype<autoware_msgs::ControlCommandStamped>(),"autoware_msgs::ControlCommandStamped topic to subscribe to.",this, SLOT(updateTopic()));
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
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_rviz_plugins::ControlCommandMonitor, rviz::Display)