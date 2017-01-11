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

// check if waypoint number is valid
bool VelocitySetPath::checkWaypoint(int num, const char *name) const
{
  if (num < 0 || num >= getSize())
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
  int size = (int)(temporal_waypoints_size / getInterval()) + 1;

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = current_waypoints_.header;
  temporal_waypoints_.increment = current_waypoints_.increment;
  // push current pose
  waypoint_follower::waypoint current_point;

  current_point.pose = control_pose;
  current_point.twist = current_waypoints_.waypoints[closest_waypoint].twist;
  current_point.dtlane = current_waypoints_.waypoints[closest_waypoint].dtlane;
  temporal_waypoints_.waypoints.push_back(current_point);
  for (int i = 0; i < size; i++)
  {
    if (closest_waypoint + i >= getSize())
      return;
    temporal_waypoints_.waypoints.push_back(current_waypoints_.waypoints[closest_waypoint + i]);
  }

  return;
}

void VelocitySetPath::setDeceleration(double current_velocity, double deceleration, int closest_waypoint)
{
  int velocity_change_range = 5;
  double intervel = getInterval();
  double temp1 = current_velocity * current_velocity;
  double temp2 = 2 * deceleration * intervel;
  double deceleration_minimum = kmph2mps(4.0);

  for (int i = 0; i < velocity_change_range; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, "setDeceleration"))
      continue;
    double waypoint_velocity = current_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x;
    double changed_vel = temp1 - temp2;
    if (changed_vel < 0)
    {
      changed_vel = deceleration_minimum * deceleration_minimum;
    }
    if (sqrt(changed_vel) > waypoint_velocity || deceleration_minimum > waypoint_velocity)
      continue;
    if (sqrt(changed_vel) < deceleration_minimum)
    {
      current_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = deceleration_minimum;
      continue;
    }
    current_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = sqrt(changed_vel);
  }

  return;
}

void VelocitySetPath::avoidSuddenAceleration(double current_velocity, double deceleration, int closest_waypoint)
{
  double changed_vel;
  double interval = getInterval();
  double temp1 = current_velocity * current_velocity;
  double temp2 = 2 * deceleration * interval;
  double velocity_offset = 1.389; // m/s

  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i, "avoidSuddenAceleration"))
      return;
    changed_vel = sqrt(temp1 + temp2 * (double)(i + 1)) + velocity_offset;
    if (changed_vel > current_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x)
      return;
    current_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = changed_vel;
  }

  return;
}

void VelocitySetPath::avoidSuddenBraking(double velocity_change_limit, double current_velocity, double deceleration, int closest_waypoint)
{
  int i = 0;
  int fill_in_zero = 20;
  int fill_in_vel = 15;
  int examin_range = 1;  // need to change according to waypoint interval?
  int num;
  double interval = getInterval();
  double changed_vel;

  for (int j = -1; j < examin_range; j++)
  {
    if (!checkWaypoint(closest_waypoint + j, "avoidSuddenBraking"))
      return;
    if (getWaypointVelocityMPS(closest_waypoint + j) <
        current_velocity - velocity_change_limit)  // we must change waypoints
      break;
    if (j == examin_range - 1)  // we don't have to change waypoints
      return;
  }

  // fill in waypoints velocity behind vehicle
  for (num = closest_waypoint - 1; fill_in_vel > 0; fill_in_vel--)
  {
    if (!checkWaypoint(num - fill_in_vel, "avoidSuddenBraking"))
      continue;
    current_waypoints_.waypoints[num - fill_in_vel].twist.twist.linear.x = current_velocity;
  }

  // decelerate gradually
  double temp1 = (current_velocity - velocity_change_limit + 1.389) * (current_velocity - velocity_change_limit + 1.389);
  double temp2 = 2 * deceleration * interval;
  for (num = closest_waypoint - 1;; num++)
  {
    if (num >= getSize())
      return;
    if (!checkWaypoint(num, "avoidSuddenBraking"))
      continue;
    changed_vel = temp1 - temp2 * (double)i;  // sqrt(v^2 - 2*a*x)
    if (changed_vel <= 0)
      break;
    current_waypoints_.waypoints[num].twist.twist.linear.x = sqrt(changed_vel);

    i++;
  }

  for (int j = 0; j < fill_in_zero; j++)
  {
    if (!checkWaypoint(num + j, "avoidSuddenBraking"))
      continue;
    current_waypoints_.waypoints[num + j].twist.twist.linear.x = 0.0;
  }


  return;
}

void VelocitySetPath::changeWaypoints(int stop_waypoint, int closest_waypoint, double deceleration, const WayPoints& prev_path)
{
  int i = 0;
  int close_waypoint_threshold = 4;
  int fill_in_zero = 20;
  double changed_vel;
  double interval = getInterval();

  // change waypoints to decelerate
  for (int num = stop_waypoint; num > closest_waypoint - close_waypoint_threshold; num--)
  {
    if (!checkWaypoint(num, "changeWaypoints"))
      continue;

    changed_vel = sqrt(2.0 * deceleration * (interval * i));  // sqrt(2*a*x)

    waypoint_follower::waypoint initial_waypoint = prev_path.getCurrentWaypoints().waypoints[num];
    if (changed_vel > initial_waypoint.twist.twist.linear.x)
    {  // avoid acceleration
      current_waypoints_.waypoints[num].twist.twist.linear.x = initial_waypoint.twist.twist.linear.x;
    }
    else
    {
      current_waypoints_.waypoints[num].twist.twist.linear.x = changed_vel;
    }

    i++;
  }

  // fill in 0
  for (int j = 1; j < fill_in_zero; j++)
  {
    if (!checkWaypoint(stop_waypoint + j, "changeWaypoints"))
      continue;
    current_waypoints_.waypoints[stop_waypoint + j].twist.twist.linear.x = 0.0;
  }


  return;
}

