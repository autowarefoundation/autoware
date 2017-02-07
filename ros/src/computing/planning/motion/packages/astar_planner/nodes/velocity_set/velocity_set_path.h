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

#ifndef VELOCITY_SET_PATH_H
#define VELOCITY_SET_PATH_H

#include "waypoint_follower/libwaypoint_follower.h"
class VelocitySetPath
{
 private:
  waypoint_follower::lane prev_waypoints_;
  waypoint_follower::lane new_waypoints_;
  waypoint_follower::lane temporal_waypoints_;
  bool set_path_;
  double current_vel_;

  // ROS param
  double velocity_offset_; // m/s

  bool checkWaypoint(int num, const char *name) const;

 public:
  VelocitySetPath();
  ~VelocitySetPath();

  void changeWaypoints(int stop_waypoint, int closest_waypoint, double deceleration);
  void avoidSuddenBraking(double velocity_change_limit, double deceleration, int closest_waypoint);
  void avoidSuddenAceleration(double decelerationint, int closest_waypoint);
  void setDeceleration(double deceleration, int closest_waypoint);
  void setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint, geometry_msgs::PoseStamped control_pose);
  void initializeNewWaypoints();

  // ROS Callbacks
  void waypointsCallback(const waypoint_follower::laneConstPtr& msg);
  void currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg);

  double calcInterval(const int begin, const int end) const;

  waypoint_follower::lane getPrevWaypoints() const
  {
    return prev_waypoints_;
  }

  waypoint_follower::lane getNewWaypoints() const
  {
    return new_waypoints_;
  }

  waypoint_follower::lane getTemporalWaypoints() const
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
};

#endif // VELOCITY_SET_PATH_H
