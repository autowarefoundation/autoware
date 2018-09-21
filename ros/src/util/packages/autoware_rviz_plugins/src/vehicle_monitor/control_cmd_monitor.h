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

//headers in boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

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
        boost::shared_ptr<rviz::RosTopicProperty> topic_property_;
        boost::shared_ptr<rviz::IntProperty> top_property_;
        boost::shared_ptr<rviz::IntProperty> left_property_;
        boost::shared_ptr<rviz::FloatProperty> alpha_property_;
        ros::Subscriber sub_;
        ros::NodeHandle nh_;
        boost::mutex mutex_;
        int monitor_top_,monitor_left_;
        float alpha_;
    protected Q_SLOTS:
        void update_topic_();
        void update_top_();
        void update_left_();
        void update_alpha_();
    };
}

#endif  //CONTROL_CMD_MONITOR_H_INCLUDED