//
// Created by kosuke on 11/29/17.
//

#ifndef OBJECT_TRACKING_BOX_FITTING_H
#define OBJECT_TRACKING_BOX_FITTING_H

#include <ros/ros.h>

#include <array>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <vector>

#include "autoware_msgs/CloudClusterArray.h"


class ClusterFilter {
private:
  float sensor_height_;
  float roi_m_;
  float pic_scale_; // picScale * roiM = 30 * 30
  // const float picScale = 30;
  int ram_points_;
  int slope_dist_thres_;
  int num_points_thres_;

  float height_min_;
  float height_max_;
  float width_min_;
  float width_max_;
  float len_min_;
  float len_max_;
  float area_max_;
  float ratio_min_;
  float ratio_max_;
  float min_len_ratio_;
  float pt_num_per_vol_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_cloud_array_;
  ros::Publisher pub_cloud_array_;

  void callBack(const autoware_msgs::CloudClusterArray& input);
  void updateCpFromPoints(const std::vector<cv::Point2f>& pc_points,
                          autoware_msgs::CloudCluster &cluster);
  void toRightAngleBBox(std::vector<cv::Point2f> &pc_points);
  void updateDimentionAndEstimatedAngle(const std::vector<cv::Point2f>& pcPoints,
                                        autoware_msgs::CloudCluster &cluster);
  void getPointsInPcFrame(cv::Point2f rect_points[],
                          std::vector<cv::Point2f> &pc_points, int offset_x,
                          int offset_y);
  bool ruleBasedFilter(std::vector<cv::Point2f> pc_points, float max_z,
                       int num_points);
  void getBBoxes(autoware_msgs::CloudClusterArray& in_cluster_array,
                 autoware_msgs::CloudClusterArray& out_cluster_array);

public:
  ClusterFilter();
};

#endif // OBJECT_TRACKING_BOX_FITTING_H
