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

#include "velocity_set_info.h"

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
    use_obstacle_sim_(false),
    wpidx_detectionResultByOtherNodes_(-1)
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("remove_points_upto", remove_points_upto_, 2.3);
}

VelocitySetInfo::~VelocitySetInfo()
{
}

void VelocitySetInfo::clearPoints()
{
  points_.clear();
}

void VelocitySetInfo::configCallback(const autoware_msgs::ConfigVelocitySetConstPtr &config)
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

  if (use_obstacle_sim_)
  {
    joinPoints(obstacle_sim_points_, &points_);
    obstacle_sim_points_.clear();
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
  localizer_pose_ = *msg;
}

void VelocitySetInfo::obstacleSimCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, obstacle_sim_points_);

  use_obstacle_sim_ = true;
}
