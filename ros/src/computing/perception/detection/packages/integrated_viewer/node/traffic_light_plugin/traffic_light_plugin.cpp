#include <ros/ros.h>
#include <runtime_manager/traffic_light.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <QString>
#include <QImage>

#include "traffic_light_plugin.h"

namespace integrated_viewer
{
  TrafficLightPlugin::TrafficLightPlugin(QWidget* parent)
    : rviz::Panel(parent) {

    // Initialize Form
    ui_.setupUi(this);

    // Set initial state to "UNDEFINED"
    StateInfo initial_info = {};
    GetStateInfo(StateNumber::UNDEFINED, initial_info);
    SetStateInfo(initial_info);

    // Boot Callback function
    signal_state_sub_ = node_handle_.subscribe("/light_color",
                                               1,
                                               &TrafficLightPlugin::SignalStateCallback,
                                               this);

  } // TrafficLightPlugin::TrafficLightPlugin()


  void TrafficLightPlugin::SignalStateCallback(const runtime_manager::traffic_light::ConstPtr& msg) {
    StateInfo info = {};
    GetStateInfo(static_cast<StateNumber>(msg->traffic_light), info);
    SetStateInfo(info);
  } // TrafficLightPlugin::SignalStateCallback()


  void TrafficLightPlugin::GetStateInfo(const StateNumber num,
                                        StateInfo& info) {
    // Return the string and color value corresponding to input 
    switch (num) {
    case StateNumber::RED: {
      info.label = "RED";
      info.label_color = "#FF0000"; // format = #RRGGBB
      info.view_color = CV_RGB(255, 0, 0);
      break;
    }
    case StateNumber::GREEN: {
      info.label = "GREEN";
      info.label_color = "#00FF00"; // format = #RRGGBB
      info.view_color = CV_RGB(0, 255, 0);
      break;
    }
    case StateNumber::UNDEFINED: {
      info.label = "NO SIGNAL FOUND";
      info.label_color = "#FFFFFF"; // white
      info.view_color = CV_RGB(0, 0, 0); // black
      break;
    }
    }

    return;
  } // TrafficLightPlugin::GetStateInfo()


  void TrafficLightPlugin::SetStateInfo(const StateInfo info) {
    // set label string to the UI
    QPalette* palette = new QPalette();
    palette->setColor(QPalette::Foreground, info.label_color);
    ui_.state_label_->setPalette(*palette); // reflect text color
    ui_.state_label_->setText(info.label);  // reflect text

    // The image to be shown on the UI
    cv::Mat circle_mat(DEFAULT_WINDOW_SIZE,
                       DEFAULT_WINDOW_SIZE,
                       CV_8UC3,
                       cv::Scalar(0));

    // Draw the circle with corresponding color of recognition state
    cv::circle(circle_mat,
               cv::Point(DEFAULT_WINDOW_SIZE/2, DEFAULT_WINDOW_SIZE/2),
               DEFAULT_RADIUS,
               info.view_color,
               CV_FILLED);      // draw circle

    viewed_image_ = convert_image::CvMatToQPixmap(circle_mat);

    // set color of circle view on the UI
    int height = ui_.state_view_->height();
    int width  = ui_.state_view_->width();
    ui_.state_view_->setPixmap(viewed_image_.scaled(width,
                                                    height,
                                                    Qt::KeepAspectRatio,
                                                    Qt::SmoothTransformation));
    
  } // TrafficLightPlugin::SetStateInfo()


} // end namespace integrated_viewer


// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(integrated_viewer::TrafficLightPlugin, rviz::Panel)
// END_TUTORIAL











