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

#include "../../src/velocity_set/velocity_set_info.h"

void joinPoints(const pcl::PointCloud<pcl::PointXYZ>& points1, pcl::PointCloud<pcl::PointXYZ>* points2)
{
  for (const auto& p : points1)
  {
    points2->push_back(p);
  }
}

VelocitySetInfo::VelocitySetInfo()
  : stop_range_(1.3),
    deceleration_range_(0),
    points_threshold_(10),
    detection_height_top_(0.2),
    detection_height_bottom_(-1.7),
    stop_distance_obstacle_(10),
    stop_distance_stopline_(5),
    deceleration_obstacle_(0.8),
    deceleration_stopline_(0.6),
    velocity_change_limit_(2.77),
    temporal_waypoints_size_(100),
    set_pose_(false),
    wpidx_detectionResultByOtherNodes_(-1)
{
  ros::NodeHandle private_nh_("~");
  ros::NodeHandle nh;
  private_nh_.param<double>("remove_points_upto", remove_points_upto_, 2.3);
  node_status_publisher_ptr_ = std::make_shared<autoware_health_checker::NodeStatusPublisher>(nh,private_nh_);
  node_status_publisher_ptr_->ENABLE();
}

VelocitySetInfo::~VelocitySetInfo()
{
}

void VelocitySetInfo::clearPoints()
{
  points_.clear();
}

void VelocitySetInfo::configCallback(const autoware_config_msgs::ConfigVelocitySetConstPtr &config)
{
  stop_distance_obstacle_ = config->stop_distance_obstacle;
  stop_distance_stopline_ = config->stop_distance_stopline;
  stop_range_ = config->detection_range;
  points_threshold_ = config->threshold_points;
  detection_height_top_ = config->detection_height_top;
  detection_height_bottom_ = config->detection_height_bottom;
  deceleration_obstacle_ = config->deceleration_obstacle;
  deceleration_stopline_ = config->deceleration_stopline;
  velocity_change_limit_ = config->velocity_change_limit / 3.6; // kmph -> mps
  deceleration_range_ = config->deceleration_range;
  temporal_waypoints_size_ = config->temporal_waypoints_size;
}

void VelocitySetInfo::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  node_status_publisher_ptr_->CHECK_RATE("/topic/rate/points_no_ground/slow",8,5,1,"topic points_no_ground subscribe rate low.");
  pcl::PointCloud<pcl::PointXYZ> sub_points;
  pcl::fromROSMsg(*msg, sub_points);

  points_.clear();
  for (const auto &v : sub_points)
  {
    if (v.x == 0 && v.y == 0)
      continue;

    if (v.z > detection_height_top_ || v.z < detection_height_bottom_)
      continue;

    // ignore points nearby the vehicle
    if (v.x * v.x + v.y * v.y < remove_points_upto_ * remove_points_upto_)
      continue;

    points_.push_back(v);
  }

}

void VelocitySetInfo::detectionCallback(const std_msgs::Int32 &msg)
{
    wpidx_detectionResultByOtherNodes_ = msg.data;
}

void VelocitySetInfo::controlPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  control_pose_ = *msg;

  if (!set_pose_)
    set_pose_ = true;
}

void VelocitySetInfo::localizerPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  node_status_publisher_ptr_->NODE_ACTIVATE();
  node_status_publisher_ptr_->CHECK_RATE("/topic/rate/current_pose/slow",8,5,1,"topic current_pose subscribe rate low.");
  localizer_pose_ = *msg;
}
