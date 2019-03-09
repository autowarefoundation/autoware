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

#ifndef __MOTION_MONITOR_H__
#define __MOTION_MONITOR_H__

#include <ros/ros.h>
#include <autoware_health_checker/health_checker/health_checker.h>
#include "motion_dataset.h"

class MotionMonitor
{
  public:
    MotionMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    void run();

    void laneCallback(const autoware_msgs::Lane::ConstPtr& lane);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
    void locationCallback(const autoware_msgs::VehicleLocation::ConstPtr& loc);

    void publish();

  private:
    std::shared_ptr<autoware_health_checker::HealthChecker> health_checker_ptr_;
    ros::Publisher deviation_pub_, distance_pub_;
    ros::Subscriber base_waypoints_sub_, location_sub_, current_pose_sub_;
    MotionDataset dataset_;
    int search_width_;
    std::array<double, 3> dev_level_default_, dist_level_default_;
};

#endif
