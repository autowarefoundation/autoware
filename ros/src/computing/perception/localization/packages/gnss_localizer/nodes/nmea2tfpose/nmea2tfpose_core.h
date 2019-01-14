/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef NMEA2TFPOSE_CORE_H
#define NMEA2TFPOSE_CORE_H

// C++ includes
#include <string>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>

#include <gnss/geo_pos_conv.hpp>

namespace gnss_localizer
{
class Nmea2TFPoseNode
{
public:
  Nmea2TFPoseNode();
  ~Nmea2TFPoseNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_;

  // subscriber
  ros::Subscriber sub1_;

  // constants
  const std::string MAP_FRAME_;
  const std::string GPS_FRAME_;

  // variables
  int32_t plane_number_;
  geo_pos_conv geo_;
  geo_pos_conv last_geo_;
  double roll_, pitch_, yaw_;
  double orientation_time_, position_time_;
  ros::Time current_time_, orientation_stamp_;
  tf::TransformBroadcaster br_;

  // callbacks
  void callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg);

  // initializer
  void initForROS();

  // functions
  void publishPoseStamped();
  void publishTF();
  void createOrientation();
  void convert(std::vector<std::string> nmea, ros::Time current_stamp);
};

std::vector<std::string> split(const std::string &string);

}  // gnss_localizer
#endif  // NMEA2TFPOSE_CORE_H
