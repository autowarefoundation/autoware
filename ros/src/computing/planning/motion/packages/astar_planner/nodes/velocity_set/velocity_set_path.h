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

#ifndef VELOCITY_SET_PATH_H
#define VELOCITY_SET_PATH_H

#include <autoware_msgs/Lane.h>
#include "waypoint_follower/libwaypoint_follower.h"
class VelocitySetPath
{
 private:
  autoware_msgs::Lane prev_waypoints_;
  autoware_msgs::Lane new_waypoints_;
  autoware_msgs::Lane temporal_waypoints_;
  bool set_path_;
  double current_vel_;

  // ROS param
  double velocity_offset_; // m/s
  double decelerate_vel_min_; // m/s

  bool checkWaypoint(int num, const char *name) const;

 public:
  VelocitySetPath();
  ~VelocitySetPath();

  void changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint, int closest_waypoint, double deceleration);
  void avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint);
  void avoidSuddenAcceleration(double decelerationint, int closest_waypoint);
  void changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint);
  void setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint, geometry_msgs::PoseStamped control_pose);
  void initializeNewWaypoints();
  void resetFlag();

  // ROS Callbacks
  void waypointsCallback(const autoware_msgs::LaneConstPtr& msg);
  void currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg);

  double calcInterval(const int begin, const int end) const;

  autoware_msgs::Lane getPrevWaypoints() const
  {
    return prev_waypoints_;
  }

  autoware_msgs::Lane getNewWaypoints() const
  {
    return new_waypoints_;
  }

  autoware_msgs::Lane getTemporalWaypoints() const
  {
    return temporal_waypoints_;
  }

  bool getSetPath() const
  {
    return set_path_;
  }

  double getCurrentVelocity() const
  {
    return current_vel_;
  }

  int getPrevWaypointsSize() const
  {
    return prev_waypoints_.waypoints.size();
  }  

  int getNewWaypointsSize() const
  {
    return new_waypoints_.waypoints.size();
  }
};

#endif // VELOCITY_SET_PATH_H
