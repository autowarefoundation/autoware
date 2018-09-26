#ifndef VHICLE_STATE_MONITOR_H_INCLUDED
#define VHICLE_STATE_MONITOR_H_INCLUDED

//headers in autoware
#include <autoware_msgs/VehicleStatus.h>
#include "overlay_utils.h"

// headers in ROS
#include <ros/package.h>
#include <ros/ros.h>

// headers in Qt
#include <QWidget>
#include <QPainter>
#include <QImage>
#include <QRect>

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

//headers in STL
#include <math.h>

#define KM_PER_HOUR 0
#define M_PER_SEC 1

#define RAD 0
#define DEG 1

namespace autoware_rviz_plugins {
    class VehicleStatusMonitor : public rviz::Display{
    Q_OBJECT
    public:
        VehicleStatusMonitor();
        virtual ~VehicleStatusMonitor();
    protected:
        virtual void onInitialize();
        virtual void reset();
        virtual void update(float wall_dt, float ros_dt);
        virtual void onEnable();
        virtual void onDisable();
    private:
        autoware_rviz_plugins::OverlayObject::Ptr overlay_;
        void processMessage(const autoware_msgs::VehicleStatus::ConstPtr& msg);
        void draw_monitor_();
        boost::shared_ptr<rviz::RosTopicProperty> topic_property_;
        boost::shared_ptr<rviz::IntProperty> top_property_;
        boost::shared_ptr<rviz::IntProperty> left_property_;
        boost::shared_ptr<rviz::IntProperty> width_property_;
        boost::shared_ptr<rviz::FloatProperty> alpha_property_;
        boost::shared_ptr<rviz::EnumProperty> speed_unit_property_;
        boost::shared_ptr<rviz::EnumProperty> angle_unit_property_;
        boost::optional<autoware_msgs::VehicleStatus> last_command_data_;
        ros::Subscriber sub_;
        ros::NodeHandle nh_;
        boost::mutex mutex_;
        int monitor_top_,monitor_left_;
        float alpha_;
        int width_,height_;
        int speed_unit_,angle_unit_;
        std::string topic_name_;
    protected Q_SLOTS:
        void update_topic_();
        void update_top_();
        void update_left_();
        void update_alpha_();
        void update_speed_unit_();
        void update_angle_unit_();
        void update_width_();
    };
}
#endif //VHICLE_STATE_MONITOR_H_INCLUDED