/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
