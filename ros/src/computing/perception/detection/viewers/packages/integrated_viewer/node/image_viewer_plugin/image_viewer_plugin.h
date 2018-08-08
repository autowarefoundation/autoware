#ifndef IMAGE_VIEWER_PLUGIN_H
#define IMAGE_VIEWER_PLUGIN_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <rviz/panel.h>
#include "autoware_msgs/image_obj.h"
#include "autoware_msgs/image_obj_ranged.h"
#include "autoware_msgs/image_obj_tracked.h"
#include "autoware_msgs/PointsImage.h"

#include <string>
#include <map>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <QStringList>
#include <QWidget>
#include <QEvent>
#include <autoware_msgs/DetectedObjectArray.h>

#include "convert_image.h"
#include "ui_image_viewer_form.h"
#include "draw_rects.h"
#include "draw_points.h"
#include "draw_lane.h"
#endif

namespace integrated_viewer
{

  class ImageViewerPlugin: public rviz::Panel {
  Q_OBJECT
  public:
    explicit ImageViewerPlugin(QWidget* parent = 0);

    // override resize event
    virtual void resizeEvent(QResizeEvent *);
    
  protected:
    // The function to update topic list that can be selected from the UI
    void UpdateTopicList(void);

    // The event filter to catch clicking on combo box
    bool eventFilter(QObject* object, QEvent* event);
    
    // The Callback functions
    void ImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void DetectedObjCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg);
    void PointCallback(const autoware_msgs::PointsImage::ConstPtr &msg);
    void LaneCallback(const autoware_msgs::ImageLaneObjects::ConstPtr& msg);

   // The function to refrect modified image on UI
   void ShowImageOnUi(void);

    // The data type of the topic that will be shown in each combo box
    static const QString kImageDataType;
    static const QString kDetectedObjectDataTypeBase;
    static const QString kPointDataType;
    static const QString kLaneDataType;

    // The blank topic name
    static const QString kBlankTopic;

    // The ROS node handle.
    ros::NodeHandle node_handle_;

    // The ROS subscriber
    ros::Subscriber image_sub_;
    ros::Subscriber rect_sub_;
    ros::Subscriber point_sub_;
    ros::Subscriber lane_sub_;

  private:
    // The UI components
    Ui::image_viewer_form ui_;

    // The image displayed on viewer
    cv::Mat viewed_image_;
    cv::Mat default_image_;

    // Data pointer to hold subscribed data
    autoware_msgs::PointsImage::ConstPtr points_msg_;
    autoware_msgs::DetectedObjectArray::ConstPtr detected_objects_msg_;
    autoware_msgs::ImageLaneObjects::ConstPtr lane_msg_;

    // The helper-class constructor for drawing
    DrawRects rects_drawer_;
    DrawPoints points_drawer_;
    DrawLane lane_drawer_;

    // The flag to represent whether default image should be shown or not
    bool default_image_shown_;

    // The behavior definition of the UI
    private Q_SLOTS:
      // We can skip "connect" process by defining naming
      // of slot function like on_"widget_name"_"signal_name"
      void on_image_topic_combo_box__activated(int index);
      void on_rect_topic_combo_box__activated(int index);
      void on_point_topic_combo_box__activated(int index);
      void on_lane_topic_combo_box__activated(int index);

  }; // end class ImageViewerPlugin

} // end namespace integrated_viewer

#endif // IMAGE_VIEWER_PLUGIN_H
