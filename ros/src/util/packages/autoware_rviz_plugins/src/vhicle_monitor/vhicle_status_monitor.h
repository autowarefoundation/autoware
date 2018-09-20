#ifndef VHICLE_STATE_MONITOR_H_INCLUDED
#define VHICLE_STATE_MONITOR_H_INCLUDED

//headers in autoware
#include <autoware_msgs/VehicleStatus.h>
#include "vhicle_monitor_drawer.h"

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
    class VhicleStatusMonitor : public rviz::MessageFilterDisplay<autoware_msgs::VehicleStatus>{
    Q_OBJECT
    public:
        VhicleStatusMonitor();
        virtual ~VhicleStatusMonitor();
    protected:
        virtual void onInitialize();
        virtual void reset();
    private:
        void processMessage(const autoware_msgs::VehicleStatus::ConstPtr& msg);
    };
}
#endif //VHICLE_STATE_MONITOR_H_INCLUDED