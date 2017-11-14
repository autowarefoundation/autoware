#ifndef GPS_PANEL_H
#define GPS_PANEL_H

#define ROADMAP 0
#define TERRAIN 1
#define SATELLITE 2
#define HYBRID 3

#define PNG 0
#define GIF 1
#define JPEG 2

#define MAX_REQUEST_URL_LENGTH 8192

#include "overlay_utils.h"

//headers for ROS
#include <ros/ros.h>
#include <ros/package.h>

//headers for Qt
#include <QWidget>

//headers for rviz
#include <rviz/message_filter_display.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
//#include <rviz/properties/string_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/uniform_string_stream.h>

//headers for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//headers for messages
#include <sensor_msgs/NavSatFix.h>

//headers for python
#include <Python.h>

//headers for stl
#include <string>
#include <fstream>
#include <iostream>
#include <stdio.h>

//headers for boost
#include <boost/circular_buffer.hpp>

namespace autoware_rviz_plugins
{
  class OverlayGpsDisplay: public rviz::MessageFilterDisplay<sensor_msgs::NavSatFix>
  {
    Q_OBJECT
    public:
      OverlayGpsDisplay();
      virtual ~OverlayGpsDisplay();
    protected:
      virtual void onInitialize();
      virtual void reset();
    private:
      void processMessage(const sensor_msgs::NavSatFix::ConstPtr& msg);
      bool download_map(std::string request_url);
      void load_map_downloader_script();
      bool build_request_url(const sensor_msgs::NavSatFix::ConstPtr& msg, std::string& request_url);
      inline bool check_map_image_file();
      rviz::IntProperty* zoom_property_;
      rviz::IntProperty* width_property_;
      rviz::IntProperty* height_property_;
      rviz::IntProperty* scale_property_;
      rviz::IntProperty* position_x_property_;
      rviz::IntProperty* position_y_property_;
      rviz::IntProperty* messages_per_plot_property_;
      rviz::IntProperty* history_length_property_;
      rviz::FloatProperty* alpha_property_;
      //rviz::StringProperty* api_key_property_;
      rviz::EnumProperty* maptype_property_;
      boost::circular_buffer<sensor_msgs::NavSatFix> fix_buffer_;
      PyObject* map_downloader_function_;
      std::string map_image_path_,api_key_;
      OverlayObject::Ptr overlay_;
    private Q_SLOTS:
      void updateGooleMapAPIProperty();
      void updateDisplayProperty();
      void updateHistoryLength();
  };
}
#endif
