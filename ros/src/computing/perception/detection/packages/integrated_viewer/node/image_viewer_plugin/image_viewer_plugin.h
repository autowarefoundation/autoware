#ifndef IMAGE_VIEWER_PLUGIN_H
#define IMAGE_VIEWER_PLUGIN_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <rviz/panel.h>
#include <cv_tracker/image_obj.h>
#include <cv_tracker/image_obj_ranged.h>
#include <cv_tracker/image_obj_tracked.h>
#include <points2image/PointsImage.h>

#include <string>
#include <map>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <QStringList>
#include <QWidget>
#include <QEvent>

#include "convert_image.h"
#include "ui_image_viewer_form.h"
#include "draw_rects.h"
#include "draw_points.h"
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
    void ImageObjCallback(const cv_tracker::image_obj::ConstPtr& msg);
    void ImageObjRangedCallback(const cv_tracker::image_obj_ranged::ConstPtr& msg);
    void ImageObjTrackedCallback(const cv_tracker::image_obj_tracked::ConstPtr& msg);
    void PointCallback(const points2image::PointsImage::ConstPtr &msg);

   // The function to refrect modified image on UI
   void ShowImageOnUi(void);

    // The data type of the topic that will be shown in each combo box
    static const QString kImageDataType;
    static const QString kRectDataTypeBase;
    static const QString kPointDataType;

    // The blank topic name
    static const QString kBlankTopic;

    // The base topic name of detection result rectangles
    static const std::string  kRectDataTypeImageObjRanged;
    static const std::string  kRectDataTypeImageObjTracked;

    // The ROS node handle.
    ros::NodeHandle node_handle_;

    // The ROS subscriber
    ros::Subscriber image_sub_;
    ros::Subscriber rect_sub_;
    ros::Subscriber point_sub_;

  private:
    // The UI components
    Ui::image_viewer_form ui_;

    // The image displayed on viewer
    cv::Mat viewed_image_;
    cv::Mat default_image_;

    // Data pointer to hold subscribed data
    points2image::PointsImage::ConstPtr points_msg_;
    cv_tracker::image_obj::ConstPtr image_obj_msg_;
    cv_tracker::image_obj_ranged::ConstPtr image_obj_ranged_msg_;
    cv_tracker::image_obj_tracked::ConstPtr image_obj_tracked_msg_;

    // data structure to hold topic information for detection result
    std::map<std::string, std::string> rect_topic_info_;

    // The helper-class constructor for drawing
    DrawRects rects_drawer_;
    DrawPoints points_drawer_;

    // The flag to represent whether default image should be shown or not
    bool default_image_shown_;

    // The behavior definition of the UI
    private Q_SLOTS:
      // We can skip "connect" process by defining naming
      // of slot function like on_"widget_name"_"signal_name"
      void on_image_topic_combo_box__activated(int index);
      void on_rect_topic_combo_box__activated(int index);
      void on_point_topic_combo_box__activated(int index);

  }; // end class ImageViewerPlugin

} // end namespace integrated_viewer

#endif // IMAGE_VIEWER_PLUGIN_H
