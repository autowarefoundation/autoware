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

#ifndef PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_CORE_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "runtime_manager/ConfigWaypointFollower.h"
#include <visualization_msgs/Marker.h>
// C++ includes
#include <memory>
#include "waypoint_follower/libwaypoint_follower.h"


enum class Mode
{
  waypoint,
  dialog
};

namespace waypoint_follower
{
class PurePursuit
{
private:

  //constant
  const double RADIUS_MAX_;
  const double KAPPA_MIN_;

  bool linear_interpolate_;

  // config topic
  int param_flag_;              // 0 = waypoint, 1 = Dialog
  double const_lookahead_distance_;  // meter
  double initial_velocity_;     // km/h
  double lookahead_distance_calc_ratio_;
  double minimum_lookahead_distance_;  // the next waypoint must be outside of this threshold.
  double displacement_threshold_;
  double relative_angle_threshold_;

  bool waypoint_set_;
  bool pose_set_;
  bool velocity_set_;
  int num_of_next_waypoint_;
  geometry_msgs::Point position_of_next_target_;
  double lookahead_distance_;

  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped current_velocity_;
  WayPoints current_waypoints_;

  double getCmdVelocity(int waypoint) const;
  void calcLookaheadDistance(int waypoint);
  double calcCurvature(geometry_msgs::Point target) const;
  double calcRadius(geometry_msgs::Point target) const;
  bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const;
  bool verifyFollowing() const;
  geometry_msgs::Twist calcTwist(double curvature, double cmd_velocity) const;
  void getNextWaypoint();
  geometry_msgs::TwistStamped outputZero() const;
  geometry_msgs::TwistStamped outputTwist(geometry_msgs::Twist t) const;

public:
  PurePursuit(bool linear_interpolate_mode)
    : RADIUS_MAX_(9e10)
    , KAPPA_MIN_(1/RADIUS_MAX_)
    , linear_interpolate_(linear_interpolate_mode)
    , param_flag_(0)
    , const_lookahead_distance_(4.0)
    , initial_velocity_(5.0)
    , lookahead_distance_calc_ratio_(2.0)
    , minimum_lookahead_distance_(6.0)
    , displacement_threshold_(0.2)
    , relative_angle_threshold_(10)
    , waypoint_set_(false)
    , pose_set_(false)
    , velocity_set_(false)
    , num_of_next_waypoint_(-1)
    , lookahead_distance_(0)
  {
  }
  ~PurePursuit()
  {
  }

  // for ROS
  void callbackFromConfig(const runtime_manager::ConfigWaypointFollowerConstPtr &config);
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackFromWayPoints(const waypoint_follower::laneConstPtr &msg);

  // for debug
  geometry_msgs::Point getPoseOfNextWaypoint() const
  {
    return current_waypoints_.getWaypointPosition(num_of_next_waypoint_);
  }
  geometry_msgs::Point getPoseOfNextTarget() const
  {
    return position_of_next_target_;
  }
  geometry_msgs::Pose getCurrentPose() const
  {
    return current_pose_.pose;
  }

  double getLookaheadDistance() const
  {
    return lookahead_distance_;
  }
  // processing for ROS
  geometry_msgs::TwistStamped go();
};
}

#endif  // PURE_PURSUIT_CORE_H
