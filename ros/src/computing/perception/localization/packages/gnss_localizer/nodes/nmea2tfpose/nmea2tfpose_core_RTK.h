/*
// *  Copyright (c) 2015, Nagoya University
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
#include <autoware_msgs/CanInfo.h>
#include <gnss/geo_pos_conv.hpp>

namespace gnss_localizer
{

class geo_array
{
private:
    unsigned int listSize;
    unsigned counter;
    geo_pos_conv *list;
    double *yaw_list;
    double *lat_std,*lon_std,*alt_std;
    double *surface_speed;
public:
    geo_array(unsigned int arraySize);
    ~geo_array();

    void push_back(const geo_pos_conv &gpc, double latstd, double lonstd, double altstd, double surfacespeed);
    double returnAngle_movingAverage(unsigned int range);

    geo_pos_conv operator[](unsigned int index);
};

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
  ros::Publisher pub_hdt;
  ros::Publisher pub_std;
  ros::Publisher pub_surface_speed;

  // subscriber
  ros::Subscriber sub1_;
  ros::Subscriber can_info;

  // constants
  const std::string MAP_FRAME_;
  const std::string GPS_FRAME_;

  // variables
  int32_t plane_number_;
  geo_pos_conv geo_,geo2_;
  //geo_pos_conv last_geo_;
  geo_array last_geo_;
  double gphdt_value;
  double roll_, pitch_, yaw_;
  double surface_speed_;
  double orientation_time_, position_time_;
  double lat_std,lon_std,alt_std;
  ros::Time current_time_, orientation_stamp_;
  tf::TransformBroadcaster br_;

  // callbacks
  void callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg);
  void callbackFromCanInfo(const autoware_msgs::CanInfo::ConstPtr &msg);

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
