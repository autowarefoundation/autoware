/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
  int random_points_;
  float slope_dist_thres_;
  int num_points_thres_;

  float roi_m_;
  float pic_scale_;
  ros::NodeHandle node_handle_;
  ros::Subscriber sub_object_array_;
  ros::Publisher pub_object_array_;

  void callback(const autoware_msgs::DetectedObjectArray& input);
  void updateCpFromPoints(const std::vector<cv::Point2f>& pointcloud_frame_points,
                          autoware_msgs::DetectedObject& output);
  void toRightAngleBBox(std::vector<cv::Point2f>& pointcloud_frame_points);
  void updateDimentionAndEstimatedAngle(const std::vector<cv::Point2f>& pcPoints,
                                        autoware_msgs::DetectedObject& object);
  void getPointsInPointcloudFrame(cv::Point2f rect_points[], std::vector<cv::Point2f>& pointcloud_frame_points,
                                  const cv::Point& offset_point);
  void getLShapeBB(const autoware_msgs::DetectedObjectArray& in_object_array,
                   autoware_msgs::DetectedObjectArray& out_object_array);

public:
  LShapeFilter();
};

#endif  // OBJECT_TRACKING_BOX_FITTING_H
