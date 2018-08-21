
#ifndef OBJECT_TRACKING_BOX_FITTING_H
#define OBJECT_TRACKING_BOX_FITTING_H

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

class LShapeFilter
{
private:

  float sensor_height_;
  int ram_points_;
  float slope_dist_thres_;
  int num_points_thres_;

  float roi_m_;
  float pic_scale_;
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_object_array_;
  ros::Publisher pub_object_array_;

  void callback(const autoware_msgs::DetectedObjectArray& input);
  void updateCpFromPoints(const std::vector<cv::Point2f>& pc_points, autoware_msgs::DetectedObject& cluster);
  void toRightAngleBBox(std::vector<cv::Point2f>& pc_points);
  void updateDimentionAndEstimatedAngle(const std::vector<cv::Point2f>& pcPoints,
                                        autoware_msgs::DetectedObject& object);
  void getPointsInPcFrame(cv::Point2f rect_points[], std::vector<cv::Point2f>& pc_points, int offset_x, int offset_y);
  bool ruleBasedFilter(std::vector<cv::Point2f> pc_points, float max_z, int num_points);
  void getLShapeBB(autoware_msgs::DetectedObjectArray& in_object_array,
                   autoware_msgs::DetectedObjectArray& out_object_array);

public:
  LShapeFilter();
};

#endif  // OBJECT_TRACKING_BOX_FITTING_H
