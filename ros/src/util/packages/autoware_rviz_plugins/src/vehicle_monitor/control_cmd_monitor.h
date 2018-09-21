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
#include <boost/optional.hpp>

namespace autoware_rviz_plugins {
    class ControlCommandMonitor : public rviz::Display{
    Q_OBJECT
    public:
        ControlCommandMonitor();
        virtual ~ControlCommandMonitor();
    protected:
        virtual void onInitialize();
        virtual void reset();
        virtual void update(float wall_dt, float ros_dt);
    private:
        void processMessage(const autoware_msgs::ControlCommandStamped::ConstPtr& msg);
        void draw_monitor_();
        boost::shared_ptr<rviz::RosTopicProperty> topic_property_;
        boost::shared_ptr<rviz::IntProperty> top_property_;
        boost::shared_ptr<rviz::IntProperty> left_property_;
        boost::shared_ptr<rviz::FloatProperty> alpha_property_;
        boost::optional<autoware_msgs::ControlCommandStamped> last_command_data_;
        ros::Subscriber sub_;
        ros::NodeHandle nh_;
        boost::mutex mutex_;
        int monitor_top_,monitor_left_;
        float alpha_;
        double warn_threshold_;
        double error_threshold_;
    protected Q_SLOTS:
        void update_topic_();
        void update_top_();
        void update_left_();
        void update_alpha_();
    };
}

#endif  //CONTROL_CMD_MONITOR_H_INCLUDED