#ifndef VISION_BEYOND_TRACK_H
#define VISION_BEYOND_TRACK_H

#include "detection.h"
#include "hungarian.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/ConfigSsd.h>
#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

//#include <rect_class_score.h>

using namespace std;

#define __APP_NAME__ "vision_beyond_track"

namespace beyondtrack {
  class BeyondTracker {
  private:
    int global_id = 1;
    bool initialized = false;
    vector<Detection> cur_det;
    vector<Detection> prev_det;
    cv::Mat cur_pose;
    cv::Mat prev_pose;
    double all_wts[4] = {0.6, 0.4, 0.2, 0.0};

    cv::Mat k;
    cv::Mat inv_k;
    cv::Mat motion;
    cv::Mat canonicalCuboid = create_cuboid();

    void initialize(cv::Mat n, double h);

    cv::Mat create_cuboid();

    void propagate_detections(cv::Mat n, double h);

    vector<vector<double>> generateScoreMatrices();

  public:

    BeyondTracker() {};

    BeyondTracker(cv::Mat k_);

    ~BeyondTracker() {};

    void process(vector<Detection> detection, cv::Mat pose, cv::Mat n, double h);

    vector<Detection> get_results();

    double get_3d2d_score(Detection cd, Detection pd);

    double get_3d3d_score(Detection cd, Detection pd);

  };
}

class BeyondTrackerNode {
  ros::Subscriber rect_image_subscriber_;
  ros::Subscriber intrinsics_subscriber_;
  // ros::Subscriber subscriber_yolo_config_;
  ros::Publisher publisher_objects_;
  ros::NodeHandle node_handle_;

  beyondtrack::BeyondTracker tracker_;

  float score_threshold_;
  float nms_threshold_;
  double image_ratio_;//resdize ratio used to fit input image to network input size
  uint32_t image_top_bottom_border_;//black strips added to the input image to maintain aspect ratio while resizing it to fit the network input size
  uint32_t image_left_right_border_;
  std::vector<cv::Scalar> colors_;

  cv::Size image_size_;

  //void convert_rect_to_image_obj(std::vector< RectClassScore<float> >& in_objects, autoware_msgs::DetectedObjectArray& out_message);
  cv::Mat extract_mat(const sensor_msgs::ImageConstPtr& msg);
  void image_callback(const sensor_msgs::ImageConstPtr& in_image_message);
  void intrinsics_callback(const sensor_msgs::CameraInfo &in_message);
  void config_cb(const autoware_msgs::ConfigSsd::ConstPtr& param);
public:
  void Run();
};

#endif
