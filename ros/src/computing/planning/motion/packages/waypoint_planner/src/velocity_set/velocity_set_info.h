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

#ifndef VELOCITY_SET_INFO_H
#define VELOCITY_SET_INFO_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

#include "autoware_config_msgs/ConfigVelocitySet.h"

#include <autoware_health_checker/node_status_publisher.h>
#include <memory>

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
  geometry_msgs::PoseStamped localizer_pose_;  // pose of sensor
  geometry_msgs::PoseStamped control_pose_;    // pose of base_link
  bool set_pose_;

  std::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_ptr_;

 public:
  VelocitySetInfo();
  ~VelocitySetInfo();

  // ROS Callback
  void configCallback(const autoware_config_msgs::ConfigVelocitySetConstPtr &msg);
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void controlPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void localizerPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
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
