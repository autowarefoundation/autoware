/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
