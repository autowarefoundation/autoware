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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * roi_object_filter.h
 *
 *  Created on: October, 23rd, 2018
 */

#ifndef PROJECT_ROI_OBJECT_FILTER_H
#define PROJECT_ROI_OBJECT_FILTER_H

#define __APP_NAME__ "roi_object_filter"

#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

#include "autoware_msgs/DetectedObjectArray.h"

class RosRoiObjectFilterApp
{
  ros::NodeHandle node_handle_;
  friend class RoiFilterTestClass;

  ros::Subscriber detections_range_subscriber_, wayarea_gridmap_subscriber_;
  ros::Publisher roi_objects_publisher_;

  message_filters::Subscriber<autoware_msgs::DetectedObjectArray> *detections_filter_subscriber_;

  message_filters::Subscriber<grid_map_msgs::GridMap> *gridmap_filter_subscriber_;

  typedef
  message_filters::sync_policies::ApproximateTime<autoware_msgs::DetectedObjectArray,
    grid_map_msgs::GridMap> SyncPolicyT;

  message_filters::Synchronizer<SyncPolicyT>
    *detections_synchronizer_;

  grid_map_msgs::GridMap::ConstPtr wayarea_gridmap_;
  autoware_msgs::DetectedObjectArray::ConstPtr object_detections_;

  tf::TransformListener *transform_listener_;

  std::string output_frame_;
  std::string vectormap_frame_;
  std::string gridmap_layer_;
  std::vector<std::string> exception_list_;
  std::string objects_src_topic_, wayarea_gridmap_topic_;
  std::string ros_namespace_;

  bool sync_topics_, gridmap_ready_, detections_ready_, processing_;

  int gridmap_no_road_value_;

  const int GRID_MAX_VALUE = 255;
  const int GRID_MIN_VALUE = 0;

  void InitializeRosIo(ros::NodeHandle &in_private_handle);

  void DetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_range_detections);

  void WayAreaGridMapCallback(const grid_map_msgs::GridMap::ConstPtr &message);

  bool CheckPointInGrid(const grid_map::GridMap &in_grid_map, const cv::Mat &in_grid_image,
                        const geometry_msgs::Point &in_point);

  void SyncedDetectionsCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &in_detections,
                                const grid_map_msgs::GridMap::ConstPtr &in_gridmap);

  tf::StampedTransform FindTransform(const std::string &in_target_frame, const std::string &in_source_frame);

  geometry_msgs::Point TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_tf);

public:
  void Run();

  RosRoiObjectFilterApp();
};


#endif //PROJECT_ROI_OBJECT_FILTER_H
