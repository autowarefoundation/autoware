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

#include "pure_pursuit_core.h"

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode() : private_nh_("~"), pp_(), LOOP_RATE_(30), current_velocity_(0), cmd_velocity_(0)
{
  initForROS();

  // initialize for PurePursuit
  pp_.setLinearInterpolationParameter(is_linear_interpolation_);
}

// Destructor
PurePursuitNode::~PurePursuitNode()
{
}

void PurePursuitNode::initForROS()
{
  // ros parameter settings
  private_nh_.param("is_linear_interpolation", is_linear_interpolation_, bool(true));
  // ROS_INFO_STREAM("is_linear_interpolation : " << is_linear_interpolation_);
  private_nh_.param("publishes_for_steering_robot",publishes_for_steering_robot_,bool(false));
  private_nh_.param("vehicle_info/wheel_base",wheel_base_,double(2.7));

  // setup subscriber
  sub1_ = nh_.subscribe("final_waypoints", 10, &PurePursuitNode::callbackFromWayPoints, this);
  sub2_ = nh_.subscribe("current_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);
  sub3_ = nh_.subscribe("config/waypoint_follower", 10, &PurePursuitNode::callbackFromConfig, this);
  sub4_ = nh_.subscribe("current_velocity", 10, &PurePursuitNode::callbackFromCurrentVelocity, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  pub2_ = nh_.advertise<waypoint_follower::ControlCommandStamped>("ctrl_cmd",10);
  pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0);  // debug tool
  pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
  // pub7_ = nh.advertise<std_msgs::Bool>("wf_stat", 0);
}

void PurePursuitNode::run()
{
  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();
    pub1_.publish(pp_.go());

    double kappa = 0;
    if(publishes_for_steering_robot_)
    {
      waypoint_follower::ControlCommandStamped ccs;
      ccs.header.stamp = ros::Time::now();
      if(pp_.canGetCurvature(&kappa))
      {
        ccs.cmd.linear_velocity = cmd_velocity_;
        ccs.cmd.steering_angle = convertCurvatureToSteeringAngle(wheel_base_,kappa);
      }
      else
      {
        ccs.cmd.linear_velocity = 0;
        ccs.cmd.steering_angle = 0;
      }
      pub2_.publish(ccs);
    }

    // for visualization with Rviz
    pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));
    pub13_.publish(displaySearchRadius(pp_.getCurrentPose().position, pp_.getLookaheadDistance()));
    pub12_.publish(displayNextTarget(pp_.getPoseOfNextTarget()));
    pub15_.publish(displayTrajectoryCircle(
        waypoint_follower::generateTrajectoryCircle(pp_.getPoseOfNextTarget(), pp_.getCurrentPose())));
    loop_rate.sleep();
  }
}

void PurePursuitNode::callbackFromConfig(const runtime_manager::ConfigWaypointFollowerConstPtr &config)
{
  pp_.getConfigForROS(config);
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  pp_.getCurrentPoseForROS(msg);
}

void PurePursuitNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_ = msg->twist.linear.x;
  pp_.getCurrentVelocityForROS(msg);
}

void PurePursuitNode::callbackFromWayPoints(const waypoint_follower::laneConstPtr &msg)
{
  cmd_velocity_ = msg->waypoints.at(0).twist.twist.linear.x;
  pp_.getWayPointsForROS(msg);
}

double convertCurvatureToSteeringAngle(const double &wheel_base, const double &kappa)
{
  return atan(wheel_base * kappa);
}

}  // waypoint_follower
