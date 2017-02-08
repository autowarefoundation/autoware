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
  waypoint_follower::waypoint current_point;

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

void VelocitySetPath::setDeceleration(double deceleration, int closest_waypoint)
{
  int velocity_change_range = 5;
  double intervel = calcInterval(0, 1);
  double temp1 = current_vel_ * current_vel_;
  double temp2 = 2 * deceleration * intervel;
  double deceleration_minimum = kmph2mps(4.0);

  for (int i = 0; i < velocity_change_range; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, "setDeceleration"))
      continue;
    double waypoint_velocity = prev_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x;
    double changed_vel = temp1 - temp2;
    if (changed_vel < 0)
    {
      changed_vel = deceleration_minimum * deceleration_minimum;
    }
    if (sqrt(changed_vel) > waypoint_velocity || deceleration_minimum > waypoint_velocity)
      continue;
    if (sqrt(changed_vel) < deceleration_minimum)
    {
      new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = deceleration_minimum;
      continue;
    }
    new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = sqrt(changed_vel);
  }

  return;
}

void VelocitySetPath::avoidSuddenAcceleration(double deceleration, int closest_waypoint)
{
  double square_current_vel = current_vel_ * current_vel_;

  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, "avoidSuddenAcceleration"))
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

void VelocitySetPath::avoidSuddenBraking(double velocity_change_limit, double deceleration, int closest_waypoint)
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
    if (!checkWaypoint(i, "avoidSuddenBraking"))
      return;

    // sqrt(v^2 - 2ax)
    double changed_vel = square_vel - 2 * deceleration * calcInterval(closest_waypoint, closest_waypoint + i);

    if (changed_vel < 0)
      break;

    new_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = std::sqrt(changed_vel);
  }

}

void VelocitySetPath::changeWaypoints(int stop_waypoint, int closest_waypoint, double deceleration)
{
  int i = 0;
  int close_waypoint_threshold = 4;
  int fill_in_zero = 20;
  double changed_vel;
  double interval = calcInterval(0, 1);

  // change waypoints to decelerate
  for (int num = stop_waypoint; num > closest_waypoint - close_waypoint_threshold; num--)
  {
    if (!checkWaypoint(num, "changeWaypoints"))
      continue;

    changed_vel = sqrt(2.0 * deceleration * (interval * i));  // sqrt(2*a*x)

    waypoint_follower::waypoint initial_waypoint = prev_waypoints_.waypoints[num];
    if (changed_vel > initial_waypoint.twist.twist.linear.x)
    {  // avoid acceleration
      new_waypoints_.waypoints[num].twist.twist.linear.x = initial_waypoint.twist.twist.linear.x;
    }
    else
    {
      new_waypoints_.waypoints[num].twist.twist.linear.x = changed_vel;
    }

    i++;
  }

  // fill in 0
  for (int j = 1; j < fill_in_zero; j++)
  {
    if (!checkWaypoint(stop_waypoint + j, "changeWaypoints"))
      continue;
    new_waypoints_.waypoints[stop_waypoint + j].twist.twist.linear.x = 0.0;
  }


  return;
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
    return -1;
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


void VelocitySetPath::waypointsCallback(const waypoint_follower::laneConstPtr& msg)
{
  prev_waypoints_ = *msg;
  new_waypoints_ = *msg;

  if (!set_path_)
    set_path_ = true;
}

void VelocitySetPath::currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_vel_ = msg->twist.linear.x;
}
