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

#include "velocity_set_path.h"

VelocitySetPath::VelocitySetPath()
  : set_path_(false),
    current_vel_(0)
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("velocity_offset", velocity_offset_, 1.2);
  private_nh_.param<double>("decelerate_vel_min", decelerate_vel_min_, 1.3);
}

VelocitySetPath::~VelocitySetPath()
{
}

// check if waypoint number is valid
bool VelocitySetPath::checkWaypoint(int num, const char *name) const
{
  if (num < 0 || num >= getPrevWaypointsSize())
  {
    return false;
  }
  return true;
}

// set about '_temporal_waypoints_size' meter waypoints from closest waypoint
void VelocitySetPath::setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint, geometry_msgs::PoseStamped control_pose)
{
  if (closest_waypoint < 0)
    return;

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = new_waypoints_.header;
  temporal_waypoints_.increment = new_waypoints_.increment;

  // push current pose
  autoware_msgs::waypoint current_point;
  current_point.pose = control_pose;
  current_point.twist = new_waypoints_.waypoints[closest_waypoint].twist;
  current_point.dtlane = new_waypoints_.waypoints[closest_waypoint].dtlane;
  temporal_waypoints_.waypoints.push_back(current_point);

  for (int i = 0; i < temporal_waypoints_size; i++)
  {
    if (closest_waypoint + i >= getNewWaypointsSize())
      return;

    temporal_waypoints_.waypoints.push_back(new_waypoints_.waypoints[closest_waypoint + i]);
  }

  return;
}

void VelocitySetPath::changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint)
{
  double square_vel_min = decelerate_vel_min_ * decelerate_vel_min_;
  int extra = 4; // for safety

  // decelerate with constant deceleration
  for (int index = obstacle_waypoint + extra; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index, __FUNCTION__))
      continue;

    // v = sqrt( (v0)^2 + 2ax )
    double changed_vel = std::sqrt(square_vel_min + 2.0 * deceleration * calcInterval(index, obstacle_waypoint));

    double prev_vel = prev_waypoints_.waypoints[index].twist.twist.linear.x;
    if (changed_vel > prev_vel)
    {
      new_waypoints_.waypoints[index].twist.twist.linear.x = prev_vel;
    }
    else
    {
      new_waypoints_.waypoints[index].twist.twist.linear.x = changed_vel;
    }
  }

}

void VelocitySetPath::avoidSuddenAcceleration(double deceleration, int closest_waypoint)
{
  double square_current_vel = current_vel_ * current_vel_;

  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, __FUNCTION__))
      return;

    // accelerate with constant acceleration
    // v = root((v0)^2 + 2ax)
    double changed_vel = std::sqrt(square_current_vel + 2 * deceleration * calcInterval(closest_waypoint, closest_waypoint + i)) + velocity_offset_;

    // Don't exceed original velocity
    if (changed_vel > new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x)
      return;

    new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = changed_vel;
  }

  return;
}

void VelocitySetPath::avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint)
{
  if (closest_waypoint < 0)
    return;

  // not avoid braking
  if (current_vel_ - new_waypoints_.waypoints[closest_waypoint].twist.twist.linear.x < velocity_change_limit)
    return;

  //std::cout << "avoid sudden braking!" << std::endl;

  double square_vel = (current_vel_ - velocity_change_limit) * (current_vel_ - velocity_change_limit);
  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, __FUNCTION__))
      return;

    // sqrt(v^2 - 2ax)
    double changed_vel = square_vel - 2 * deceleration * calcInterval(closest_waypoint, closest_waypoint + i);

    if (changed_vel < 0)
      break;

    new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = std::sqrt(changed_vel);
  }

}

void VelocitySetPath::changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint, int closest_waypoint, double deceleration)
{
  if (closest_waypoint < 0)
    return;

  // decelerate with constant deceleration
  for (int index = stop_waypoint; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index, __FUNCTION__))
      continue;

    // v = (v0)^2 + 2ax, and v0 = 0
    double changed_vel = std::sqrt(2.0 * deceleration * calcInterval(index, stop_waypoint));

    double prev_vel = prev_waypoints_.waypoints[index].twist.twist.linear.x;
    if (changed_vel > prev_vel)
    {
      new_waypoints_.waypoints[index].twist.twist.linear.x = prev_vel;
    }
    else
    {
      new_waypoints_.waypoints[index].twist.twist.linear.x = changed_vel;
    }
  }

  // fill velocity with 0 for stopping
  for (int i = stop_waypoint; i <= obstacle_waypoint; i++)
  {
    new_waypoints_.waypoints[i].twist.twist.linear.x = 0;
  }

}

void VelocitySetPath::initializeNewWaypoints()
{
  new_waypoints_ = prev_waypoints_;
}

double VelocitySetPath::calcInterval(const int begin, const int end) const
{
  // check index
  if (begin < 0 || begin >= getPrevWaypointsSize() || end < 0 || end >= getPrevWaypointsSize())
  {
    ROS_WARN("Invalid index");
    return 0;
  }

  // Calculate the inteval of waypoints
  double dist_sum = 0;
  for (int i = begin; i < end; i++)
  {
    tf::Vector3 v1(prev_waypoints_.waypoints[i].pose.pose.position.x,
                   prev_waypoints_.waypoints[i].pose.pose.position.y, 0);

    tf::Vector3 v2(prev_waypoints_.waypoints[i + 1].pose.pose.position.x,
                   prev_waypoints_.waypoints[i + 1].pose.pose.position.y, 0);

    dist_sum += tf::tfDistance(v1, v2);
  }

  return dist_sum;
}

void VelocitySetPath::resetFlag()
{
  set_path_ = false;
}


void VelocitySetPath::waypointsCallback(const autoware_msgs::laneConstPtr& msg)
{
  prev_waypoints_ = *msg;
  // temporary, edit waypoints velocity later
  new_waypoints_ = *msg;

  set_path_ = true;
}

void VelocitySetPath::currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_vel_ = msg->twist.linear.x;
}
