#ifndef TRAFFIC_LIGHT_PLUGIN_H
#define TRAFFIC_LIGHT_PLUGIN_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <rviz/panel.h>
#include <runtime_manager/traffic_light.h>

#include "convert_image.h"
#include "ui_form.h"
#endif

namespace integrated_viewer
{

  class TrafficLightPlugin: public rviz::Panel {
  Q_OBJECT
  public:
    explicit TrafficLightPlugin(QWidget* parent = 0);
    
  protected:
    // The Color state number definition
    enum StateNumber {
      RED       = 0,
      GREEN     = 1,
      UNDEFINED = 2,
    };

    // The structure to represent viewer information
    struct StateInfo {
      QString label;
      QColor label_color;
      cv::Scalar view_color;
    };

    // The Callback function for signal state
    void SignalStateCallback(const runtime_manager::traffic_light::ConstPtr& msg);

    // The function to convert recognition result to color and string information
    void GetStateInfo(const StateNumber num, StateInfo& info);

    // The function to reflect information structure to the UI
    void SetStateInfo(const StateInfo info);
    
    // The ROS node handle.
    ros::NodeHandle node_handle_;

    // The ROS subscriber for traffic light recognition result
    ros::Subscriber signal_state_sub_;

  private:
    // The UI components
    Ui::Form ui_;
    
    // The image displayed on viewer
    QPixmap viewed_image_;

    // The initial size of displayed image
    static const int DEFAULT_WINDOW_SIZE = 500;
    static const int DEFAULT_RADIUS = 200;

  };

} // end namespace integrated_viewer

#endif // TRAFFIC_LIGHT_PLUGIN_H
