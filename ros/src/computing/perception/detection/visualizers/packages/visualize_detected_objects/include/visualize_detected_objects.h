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

#ifndef _VISUALIZEDETECTEDOBJECTS_H
#define _VISUALIZEDETECTEDOBJECTS_H

#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <jsk_recognition_msgs/PolygonArray.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#define __APP_NAME__ "visualize_detected_objects"

class VisualizeDetectedObjects
{
private:
  const double arrow_height_;
  const double label_height_;
  const double object_max_linear_size_ = 50.;
  double object_speed_threshold_;
  double arrow_speed_threshold_;
  double marker_display_duration_;

  std::string input_topic_, ros_namespace_;

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_detected_objects_;

  ros::Publisher publisher_arrow_markers_;
  ros::Publisher publisher_label_markers_;
  ros::Publisher publisher_bounding_boxes_;
  ros::Publisher publisher_polygon_hulls_;
  ros::Publisher publisher_centroid_markers_;

  visualization_msgs::MarkerArray ObjectsToLabels(const autoware_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToArrows(const autoware_msgs::DetectedObjectArray &in_objects);

  jsk_recognition_msgs::BoundingBoxArray ObjectsToBoxes(const autoware_msgs::DetectedObjectArray &in_objects);

  jsk_recognition_msgs::PolygonArray ObjectsToHulls(const autoware_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToCentroids(const autoware_msgs::DetectedObjectArray &in_objects);

  void DetectedObjectsCallback(const autoware_msgs::DetectedObjectArray &in_objects);

  bool IsObjectValid(const autoware_msgs::DetectedObject &in_object);

public:
  VisualizeDetectedObjects();
};

#endif  // _VISUALIZEDETECTEDOBJECTS_H
