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

#include "../../src/velocity_set/velocity_set_path.h"

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
  autoware_msgs::Waypoint current_point;
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


void VelocitySetPath::waypointsCallback(const autoware_msgs::LaneConstPtr& msg)
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
