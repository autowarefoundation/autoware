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

#include "motion_monitor.h"

MotionMonitor::MotionMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
  dev_level_default_({2.0, 3.0, 3.5}),
  dist_level_default_({4.0, 6.0, 7.0}),
  search_width_(10)
{
  deviation_pub_ = nh.advertise<std_msgs::Float32>("viz/deviation_from_path", 0);
  distance_pub_ = nh.advertise<std_msgs::Float32>("viz/distance_from_nearest_point", 0);
  base_waypoints_sub_ = nh.subscribe("base_waypoints", 1, &MotionMonitor::laneCallback, this);
  closest_sub_ = nh.subscribe("closest_waypoint", 1, &MotionMonitor::closestCallback, this);
  current_pose_sub_ = nh.subscribe("current_pose", 1, &MotionMonitor::poseCallback, this);
  health_checker_ptr_ = std::make_shared<autoware_health_checker::HealthChecker>(nh, pnh);
  health_checker_ptr_->ENABLE();
  health_checker_ptr_->NODE_ACTIVATE();
}

void MotionMonitor::run()
{
  ros::Rate rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    publish();
    dataset_.reset();
    rate.sleep();
  }
}

void MotionMonitor::laneCallback(const autoware_msgs::Lane::ConstPtr& lane)
{
  dataset_.lane_ = *lane;
  dataset_.init_.at(DataType::TYPE_LANE) = true;
}

void MotionMonitor::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
  dataset_.pose_ = *pose;
  dataset_.init_.at(DataType::TYPE_POSE) = true;
}

void MotionMonitor::closestCallback(const std_msgs::Int32::ConstPtr& wpidx)
{
  dataset_.closest_idx_ = *wpidx;
  dataset_.init_.at(DataType::TYPE_IDX) = true;
}

void MotionMonitor::publish()
{
  std_msgs::Float32 dev_msg, dist_msg;
  auto& dev = dev_msg.data;
  auto& dist = dist_msg.data;

  if (!dataset_.check())
  {
    dev = dist = -1.0;
  }
  else
  {
    const auto& wp = dataset_.lane_.waypoints;
    const auto& pos = dataset_.pose_.pose.position;
    const auto& closest = dataset_.closest_idx_.data;
    dist = getPlaneDistance(wp[closest].pose.pose.position, pos);

    const int size = dataset_.getLaneSize();
    double nearest_dev = DBL_MAX;
    if (size > 1)
    {
      std::array<double, 3> coeff;
      const std::pair<int, int> idx = dataset_.calcNearestPair(search_width_);
      const auto& pnt_s = wp[idx.first].pose.pose.position;
      const auto& pnt_g = wp[idx.second].pose.pose.position;
      const bool coeff_ok =
        getLinearEquation(pnt_s, pnt_g, &coeff[0], &coeff[1], &coeff[2]);
      nearest_dev = coeff_ok ?
        getDistanceBetweenLineAndPoint(pos, coeff[0], coeff[1], coeff[2]) : nearest_dev;
    }
    dev = std::min(nearest_dev, (double)dist);
  }
  const auto& devth = dev_level_default_;
  const auto& distth = dist_level_default_;
  health_checker_ptr_->CHECK_MAX_VALUE("path_deviation_long", dev,
    devth[0], devth[1], devth[2], "deviation from path is too long");
  health_checker_ptr_->CHECK_MAX_VALUE("closest_distance_long", dist,
    distth[0], distth[1], distth[2], "distance from closest_wp is too long");
  if (dist < 0.0)
  {
    return;
  }
  deviation_pub_.publish(dev_msg);
  distance_pub_.publish(dist_msg);
}
