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

#ifndef _LIB_WAYPOINT_FOLLOWER_H
#define _LIB_WAYPOINT_FOLLOWER_H

//C++ header
#include <iostream>
#include <sstream>
#include <fstream>

//ROS header
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "waypoint_follower/lane.h"

class Path
{
protected:
	waypoint_follower::lane current_path_;
  tf::Vector3 origin_v_;
  tf::Transform transform_;
  int closest_waypoint_;


public:

  Path()
  {
    closest_waypoint_ = -1;
    origin_v_.setZero();
  }
  void setTransform(tf::Transform transform){ transform_ = transform;}
  void setPath(const waypoint_follower::laneConstPtr &msg);
  int getPathSize();
  double getInterval();
	double getDistance(int waypoint);
  tf::Vector3 transformWaypoint(int waypoint);
	geometry_msgs::Point getWaypointPosition(int waypoint);
	geometry_msgs::Quaternion getWaypointOrientation(int waypoint);
	waypoint_follower::lane getCurrentPath(){ return current_path_; }
	int getClosestWaypoint();

};

inline double kmph2mps(double velocity_kmph) { return (velocity_kmph * 1000) / (60 * 60); }
inline double mps2kmph(double velocity_mps) { return (velocity_mps * 60 * 60) / 1000; }
double DecelerateVelocity(double distance, double prev_velocity);
inline tf::Vector3 point2vector(geometry_msgs::Point point)
{
  tf::Vector3 vector(point.x,point.y,point.z);
  return vector;
}

inline geometry_msgs::Point vector2point(tf::Vector3 vector)
{
  geometry_msgs::Point point;
  point.x = vector.getX();
  point.y = vector.getY();
  point.z = vector.getZ();
  return point;
}

#endif
