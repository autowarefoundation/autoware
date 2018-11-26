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
 */

#ifndef OBJECT_TRACKING_VISUALIZEDETECTEDOBJECTS_H
#define OBJECT_TRACKING_VISUALIZEDETECTEDOBJECTS_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/io/io.h>

#include <vector>
#include <string>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

class VisualizeDetectedObjects
{
private:
  const double vis_arrow_height_;
  const double vis_id_height_;
  double ignore_velocity_thres_;
  double visualize_arrow_velocity_thres_;
  std::string input_topic_;
  std::string pointcloud_frame_;

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_object_array_;

  ros::Publisher pub_arrow_;
  ros::Publisher pub_id_;

  void visMarkers(const autoware_msgs::DetectedObjectArray& input);
  void callBack(const autoware_msgs::DetectedObjectArray& input);

public:
  VisualizeDetectedObjects();
};

#endif  // OBJECT_TRACKING_VISUALIZEDETECTEDOBJECTS_H
