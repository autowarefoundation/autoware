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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>

// User defined includes
#include "autoware_msgs/ConfigWaypointFollower.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/lane.h"
#include "pure_pursuit.h"
#include "pure_pursuit_viz.h"

namespace waypoint_follower
{
enum class Mode : int32_t
{
  waypoint,
  dialog,

  unknown = -1,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

class PurePursuitNode
{
public:
  PurePursuitNode();
  ~PurePursuitNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // class
  PurePursuit pp_;

  // publisher
  ros::Publisher pub1_, pub2_, pub11_, pub12_, pub13_, pub14_, pub15_, pub16_, pub17_;

  // subscriber
  ros::Subscriber sub1_, sub2_, sub3_, sub4_;

  // constant
  const int LOOP_RATE_;  // processing frequency

  // variables
  bool is_linear_interpolation_, publishes_for_steering_robot_;
  bool is_waypoint_set_, is_pose_set_, is_velocity_set_, is_config_set_;
  double current_linear_velocity_, command_linear_velocity_;
  double wheel_base_;

  int32_t param_flag_;               // 0 = waypoint, 1 = Dialog
  double const_lookahead_distance_;  // meter
  double const_velocity_;            // km/h
  double lookahead_distance_ratio_;
  double minimum_lookahead_distance_;  // the next waypoint must be outside of this threshold.

  // callbacks
  void callbackFromConfig(const autoware_msgs::ConfigWaypointFollowerConstPtr &config);
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackFromWayPoints(const autoware_msgs::laneConstPtr &msg);

  // initializer
  void initForROS();

  // functions
  void publishTwistStamped(const bool &can_get_curvature, const double &kappa) const;
  void publishControlCommandStamped(const bool &can_get_curvature, const double &kappa) const;
  void publishDeviationCurrentPosition(const geometry_msgs::Point &point,
                                       const std::vector<autoware_msgs::waypoint> &waypoints) const;

  double computeLookaheadDistance() const;
  double computeCommandVelocity() const;
  double computeCommandAccel() const;
  double computeAngularGravity(double velocity, double kappa) const;
};

double convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa);

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

}  // waypoint_follower

#endif  // PURE_PURSUIT_CORE_H
