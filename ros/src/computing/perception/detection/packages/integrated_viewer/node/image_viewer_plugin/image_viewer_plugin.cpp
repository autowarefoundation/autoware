#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <QString>
#include <QImage>

#include "image_viewer_plugin.h"
#include "draw_rects.h"
#include "draw_points.h"

#define XSTR(x) #x
#define STR(x) XSTR(x)

namespace integrated_viewer
{
  const QString     ImageViewerPlugin::kImageDataType               = "sensor_msgs/Image";
  const QString     ImageViewerPlugin::kRectDataTypeBase            = "cv_tracker/image_obj";
  const QString     ImageViewerPlugin::kPointDataType               = "points2image/PointsImage";
  const QString     ImageViewerPlugin::kBlankTopic                  = "-----";
  const std::string ImageViewerPlugin::kRectDataTypeImageObjRanged  = "cv_tracker/image_obj_ranged";
  const std::string ImageViewerPlugin::kRectDataTypeImageObjTracked = "cv_tracker/image_obj_tracked";

  ImageViewerPlugin::ImageViewerPlugin(QWidget* parent)
    : rviz::Panel(parent) {

    // Initialize Form
    ui_.setupUi(this);

    // Load default image
    default_image_ = cv::imread(STR(IMAGE_VIEWER_DEFAULT_IMAGE));

    points_msg_ = NULL;
    image_obj_msg_ = NULL;
    image_obj_ranged_msg_ = NULL;
    image_obj_tracked_msg_ = NULL;

    UpdateTopicList();

    viewed_image_ = default_image_.clone();
    default_image_shown_ = true;
    ShowImageOnUi();

    // If combobox is clicked, topic list will be update
    ui_.image_topic_combo_box_->installEventFilter(this);
    ui_.rect_topic_combo_box_->installEventFilter(this);
    ui_.point_topic_combo_box_->installEventFilter(this);

  } // ImageViewerPlugin::ImageViewerPlugin()


  void ImageViewerPlugin::UpdateTopicList(void) {
    // The topic list that can be selected from the UI
    QStringList image_topic_list;
    QStringList rect_topic_list;
    QStringList point_topic_list;

    // The topic name currently chosen
    QString image_topic_current = ui_.image_topic_combo_box_->currentText();
    QString rect_topic_current = ui_.rect_topic_combo_box_->currentText();
    QString point_topic_current = ui_.point_topic_combo_box_->currentText();

    if (image_topic_current == "") {
      image_topic_current = kBlankTopic;
    }

    if (rect_topic_current == "") {
      rect_topic_current = kBlankTopic;
    }

    if (point_topic_current == "") {
      point_topic_current = kBlankTopic;
    }

    // reset topic information list for detection result
    rect_topic_info_.clear();

    // Insert blank topic name to the top of the lists
    image_topic_list << kBlankTopic;
    rect_topic_list  << kBlankTopic;
    point_topic_list << kBlankTopic;

    // Get all available topic 
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    // Analyse topics
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
      const ros::master::TopicInfo& info = *it;
      const QString topic_name = QString::fromStdString(info.name);
      const QString topic_type = QString::fromStdString(info.datatype);
      
      // Check whether this topic is image
      if (topic_type.contains(kImageDataType) == true) {
        image_topic_list << topic_name;
        continue;
      }

      // Check whether this topic is rectangle
      if (topic_type.contains(kRectDataTypeBase) == true) { 
        // This condition will also be true for "image_obj_ranged"and "image_obj_tracked"
        rect_topic_list << topic_name;
        // Insert topic name and data type to a list
        rect_topic_info_[info.name] = info.datatype;
        continue;
      }

      // Check whether this topic is point cloud
      if (topic_type.contains(kPointDataType) == true) {
        point_topic_list << topic_name;
        continue;
      }
    }

    // remove all list items from combo box
    ui_.image_topic_combo_box_->clear();
    ui_.rect_topic_combo_box_->clear();
    ui_.point_topic_combo_box_->clear();

    // set new items to combo box
    ui_.image_topic_combo_box_->addItems(image_topic_list);
    ui_.rect_topic_combo_box_->addItems(rect_topic_list);
    ui_.point_topic_combo_box_->addItems(point_topic_list);
   
    ui_.image_topic_combo_box_->insertSeparator(1);
    ui_.rect_topic_combo_box_->insertSeparator(1);
    ui_.point_topic_combo_box_->insertSeparator(1);

    // set last topic as current
    int image_topic_index = ui_.image_topic_combo_box_->findText(image_topic_current);
    int rect_topic_index = ui_.rect_topic_combo_box_->findText(rect_topic_current);
    int point_topic_index = ui_.point_topic_combo_box_->findText(point_topic_current);

    if (image_topic_index != -1) {
      ui_.image_topic_combo_box_->setCurrentIndex(image_topic_index);
    }

    if (rect_topic_index != -1) {
      ui_.rect_topic_combo_box_->setCurrentIndex(rect_topic_index);
    }

    if (point_topic_index != -1) {
      ui_.point_topic_combo_box_->setCurrentIndex(point_topic_index);
    }

  } // ImageViewerPlugin::UpdateTopicList()





  // The behavior of combo box for image
  void ImageViewerPlugin::on_image_topic_combo_box__activated(int index) {
    // Extract selected topic name from combo box
    std::string selected_topic = ui_.image_topic_combo_box_->itemText(index).toStdString();
    if (selected_topic == kBlankTopic.toStdString() || selected_topic == "") {
      image_sub_.shutdown();
      // If blank name is selected as image topic, show default image
      viewed_image_ = default_image_.clone();
      default_image_shown_ = true;
      ShowImageOnUi();
      return;
    }
    
    // if selected topic is not blank or empty, start callback function
    default_image_shown_ = false;
    image_sub_ = node_handle_.subscribe<sensor_msgs::Image>(selected_topic,
                                                            1,
                                                            &ImageViewerPlugin::ImageCallback,
                                                            this);

  } // ImageViewerPlugin::on_image_topic_combo_box__activated()


  void ImageViewerPlugin::ImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // Get image from topic
    const auto& encoding = sensor_msgs::image_encodings::BGR8;
    viewed_image_ = cv_bridge::toCvCopy(msg, encoding)->image;

    ShowImageOnUi();
  } // ImageViewerPlugin::ImageCallback()


  // The behavior of combo box for detection result rectangle
  void ImageViewerPlugin::on_rect_topic_combo_box__activated(int index) {
    // Extract selected topic name from combo box
    std::string selected_topic = ui_.rect_topic_combo_box_->itemText(index).toStdString();
    if (selected_topic == kBlankTopic.toStdString() || selected_topic == "") {
      rect_sub_.shutdown();
      image_obj_msg_         = NULL;
      image_obj_ranged_msg_  = NULL;
      image_obj_tracked_msg_ = NULL;
      return;
    }

    // Switch booted callback function by topic name 
    std::string topic_type = rect_topic_info_[selected_topic];
    if (topic_type.find(kRectDataTypeImageObjRanged) != std::string::npos) {
      image_obj_msg_         = NULL;
      image_obj_ranged_msg_  = NULL;
      image_obj_tracked_msg_ = NULL;
      // this topic type is image_obj_ranged
      rect_sub_ = node_handle_.subscribe<cv_tracker::image_obj_ranged>(selected_topic,
                                                                       1,
                                                                       &ImageViewerPlugin::ImageObjRangedCallback,
                                                                       this);

    } else if (topic_type.find(kRectDataTypeImageObjTracked) != std::string::npos) {
      image_obj_msg_         = NULL;
      image_obj_ranged_msg_  = NULL;
      image_obj_tracked_msg_ = NULL;
      // this topic type is image_obj_tracked
      rect_sub_ = node_handle_.subscribe<cv_tracker::image_obj_tracked>(selected_topic,
                                                                       1,
                                                                       &ImageViewerPlugin::ImageObjTrackedCallback,
                                                                       this);
    } else {
      image_obj_msg_         = NULL;
      image_obj_ranged_msg_  = NULL;
      image_obj_tracked_msg_ = NULL;
      // this topic type is image_obj
      rect_sub_ = node_handle_.subscribe<cv_tracker::image_obj>(selected_topic,
                                                                1,
                                                                &ImageViewerPlugin::ImageObjCallback,
                                                                this);
    }


  } // ImageViewerPlugin::on_rect_topic_combo_box__activated()
  

  void ImageViewerPlugin::ImageObjCallback(const cv_tracker::image_obj::ConstPtr& msg) {
    image_obj_msg_ = msg;
  } // ImageViewerPlugin::ImageObjCallback()

  void ImageViewerPlugin::ImageObjRangedCallback(const cv_tracker::image_obj_ranged::ConstPtr &msg) {
    image_obj_ranged_msg_ = msg;
  } // ImageViewerPlugin::ImageObjRangedCallback()

  void ImageViewerPlugin::ImageObjTrackedCallback(const cv_tracker::image_obj_tracked::ConstPtr &msg) {
    image_obj_tracked_msg_ = msg;
  } // ImageViewerPlugin::ImageObjTrackedCallback()


  // The behavior of combo box for points image
  void ImageViewerPlugin::on_point_topic_combo_box__activated(int index) {
    // Extract selected topic name from combo box
    std::string selected_topic = ui_.point_topic_combo_box_->itemText(index).toStdString();
    if (selected_topic == kBlankTopic.toStdString() || selected_topic == "") {
      point_sub_.shutdown();
      points_msg_ = NULL;
      return;
    }

    // if selected topic is not blank or empty , start callback function
    point_sub_ = node_handle_.subscribe<points2image::PointsImage>(selected_topic,
                                                                   1,
                                                                   &ImageViewerPlugin::PointCallback,
                                                                   this);

  } // ImageViewerPlugin::on_point_topic_combo_box__activated()


  void ImageViewerPlugin::PointCallback(const points2image::PointsImage::ConstPtr &msg) {
    points_msg_ = msg;
  } // ImageViewerPlugin::PointCallback()


  void ImageViewerPlugin::ShowImageOnUi(void) {
    // Additional things will be drawn if shown image is not default one
    if (!default_image_shown_) {
      // Draw detection result rectangles on the image
      rects_drawer_.DrawImageObj(image_obj_msg_, viewed_image_);
      rects_drawer_.DrawImageObjRanged(image_obj_ranged_msg_, viewed_image_);
      rects_drawer_.DrawImageObjTracked(image_obj_tracked_msg_, viewed_image_);

      // Draw points on the image
      points_drawer_.Draw(points_msg_, viewed_image_);
    }
    // Convert cv::Mat to QPixmap to show modified image on the UI
    QPixmap view_on_ui = convert_image::CvMatToQPixmap(viewed_image_);

    // Reflect image on UI
    int height = ui_.view_->height();
    int width  = ui_.view_->width();
    ui_.view_->setPixmap(view_on_ui.scaled(width,
                                           height,
                                           Qt::KeepAspectRatio,
                                           Qt::SmoothTransformation));
  } // ImageViewerPlugin::ShowImageOnUi()


  void ImageViewerPlugin::resizeEvent(QResizeEvent *) {
    ShowImageOnUi();
  } // ImageViewerPlugin::resizeEvent()


  bool ImageViewerPlugin::eventFilter(QObject* object, QEvent* event) {
    if (event->type() == QEvent::MouseButtonPress) {
      // combo box will update its contents if this filter is applied
      UpdateTopicList();
    }

    return QObject::eventFilter(object, event);
  }


} // end namespace integrated_viewer


// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(integrated_viewer::ImageViewerPlugin, rviz::Panel)
// END_TUTORIAL
