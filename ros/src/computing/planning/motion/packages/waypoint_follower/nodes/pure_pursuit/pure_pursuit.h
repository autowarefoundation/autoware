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

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// User defined includes
#include "autoware_msgs/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"

namespace waypoint_follower
{
class PurePursuit
{
public:
  PurePursuit();
  ~PurePursuit();

  // for setting data
  void setLookaheadDistance(const double &ld)
  {
    lookahead_distance_ = ld;
  }
  void setCurrentVelocity(const double &cur_vel)
  {
    current_linear_velocity_ = cur_vel;
  }
  void setCurrentWaypoints(const std::vector<autoware_msgs::waypoint> &wps)
  {
    current_waypoints_ = wps;
  }
  void setCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    current_pose_ = msg->pose;
  }
  void setLinearInterpolationParameter(const bool &param)
  {
    is_linear_interpolation_ = param;
  }

  // for debug on ROS
  geometry_msgs::Point getPoseOfNextWaypoint() const
  {
    return current_waypoints_.at(next_waypoint_number_).pose.pose.position;
  }
  geometry_msgs::Point getPoseOfNextTarget() const
  {
    return next_target_position_;
  }
  geometry_msgs::Pose getCurrentPose() const
  {
    return current_pose_;
  }
  double getLookaheadDistance() const
  {
    return lookahead_distance_;
  }
  // processing
  bool canGetCurvature(double *output_kappa);

private:
  // constant
  const double RADIUS_MAX_;
  const double KAPPA_MIN_;

  // variables
  bool is_linear_interpolation_;
  int next_waypoint_number_;
  geometry_msgs::Point next_target_position_;
  double lookahead_distance_;
  geometry_msgs::Pose current_pose_;
  double current_linear_velocity_;
  std::vector<autoware_msgs::waypoint> current_waypoints_;

  // functions
  double calcCurvature(geometry_msgs::Point target) const;
  bool interpolateNextTarget(int next_waypoint, geometry_msgs::Point *next_target) const;
  void getNextWaypoint();
};
}  // waypoint_follower

#endif  // PURE_PURSUIT_H
