#ifndef CONTROL_CMD_MONITOR_H_INCLUDED
#define CONTROL_CMD_MONITOR_H_INCLUDED

//headers in autoware
#include <autoware_msgs/ControlCommandStamped.h>
#include "monitor_drawer.h"

// headers in ROS
#include <ros/package.h>
#include <ros/ros.h>

// headers for opencv
#include <opencv2/core/core.hpp>

// headers in Qt
#include <QWidget>

// headers in rviz
#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
//#include <rviz/properties/string_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/uniform_string_stream.h>

namespace autoware_rviz_plugins {
    class ControlCommandMonitor : public rviz::Display{
    Q_OBJECT
    public:
        ControlCommandMonitor();
        virtual ~ControlCommandMonitor();
    protected:
        virtual void onInitialize();
        virtual void reset();
    private:
        void processMessage(const autoware_msgs::ControlCommandStamped::ConstPtr& msg);
        rviz::RosTopicProperty* update_topic_property_;
    protected Q_SLOTS:
        void update_topic_();
    };
}

#endif  //CONTROL_CMD_MONITOR_H_INCLUDED