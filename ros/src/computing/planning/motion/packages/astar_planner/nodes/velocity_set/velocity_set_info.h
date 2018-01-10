/*
 *  Copyright (c) 2015, Nagoya University
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

#ifndef VELOCITY_SET_INFO_H
#define VELOCITY_SET_INFO_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

#include "autoware_msgs/ConfigVelocitySet.h"

class VelocitySetInfo
{
 private:
  // parameters
  double stop_range_;               // if obstacle is in this range, stop
  double deceleration_range_;       // if obstacle is in this range, decelerate
  int points_threshold_;            // points threshold to find obstacles
  double detection_height_top_;     // from sensor
  double detection_height_bottom_;  // from sensor
  double stop_distance_obstacle_;   // (meter) stopping distance from obstacles
  double stop_distance_stopline_;   // (meter) stopping distance from stoplines
  double deceleration_obstacle_;    // (m/s^2) deceleration for obstacles
  double deceleration_stopline_;    // (m/s^2) deceleration for stopline
  double velocity_change_limit_;    // (m/s)
  double temporal_waypoints_size_;  // (meter)
  int	wpidx_detectionResultByOtherNodes_; // waypoints index@finalwaypoints

  // ROS param
  double remove_points_upto_;

  pcl::PointCloud<pcl::PointXYZ> points_;
  pcl::PointCloud<pcl::PointXYZ> obstacle_sim_points_;
  geometry_msgs::PoseStamped localizer_pose_;  // pose of sensor
  geometry_msgs::PoseStamped control_pose_;    // pose of base_link
  bool set_pose_;
  bool use_obstacle_sim_;

 public:
  VelocitySetInfo();
  ~VelocitySetInfo();

  // ROS Callback
  void configCallback(const autoware_msgs::ConfigVelocitySetConstPtr &msg);
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void controlPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void localizerPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void obstacleSimCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void detectionCallback(const std_msgs::Int32 &msg);

  void clearPoints();


  int getDetectionResultByOtherNodes() const
  {
	  return wpidx_detectionResultByOtherNodes_;
  }

  double getStopRange() const
  {
    return stop_range_;
  }

  double getDecelerationRange() const
  {
    return deceleration_range_;
  }

  int getPointsThreshold() const
  {
    return points_threshold_;
  }

  int getDetectionHeightTop() const
  {
    return detection_height_top_;
  }

  int getDetectionHeightBottom() const
  {
    return detection_height_bottom_;
  }

  int getStopDistanceObstacle() const
  {
    return stop_distance_obstacle_;
  }

  int getStopDistanceStopline() const
  {
    return stop_distance_stopline_;
  }

  double getDecelerationObstacle() const
  {
    return deceleration_obstacle_;
  }

  double getDecelerationStopline() const
  {
    return deceleration_stopline_;
  }

  double getVelocityChangeLimit() const
  {
    return velocity_change_limit_;
  }

  double getTemporalWaypointsSize() const
  {
    return temporal_waypoints_size_;
  }

  pcl::PointCloud<pcl::PointXYZ> getPoints() const
  {
    return points_;
  }

  geometry_msgs::PoseStamped getControlPose() const
  {
    return control_pose_;
  }

  geometry_msgs::PoseStamped getLocalizerPose() const
  {
    return localizer_pose_;
  }

  bool getSetPose() const
  {
    return set_pose_;
  }
};

#endif // VELOCITY_SET_INFO_H
