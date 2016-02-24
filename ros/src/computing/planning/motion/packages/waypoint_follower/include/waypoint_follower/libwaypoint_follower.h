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

#ifndef _LIB_WAYPOINT_FOLLOWER_H_
#define _LIB_WAYPOINT_FOLLOWER_H_

//C++ header
#include <iostream>
#include <sstream>
#include <fstream>

//ROS header
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "waypoint_follower/lane.h"

class WayPoints
{
protected:
	waypoint_follower::lane current_waypoints_;

public:
	void setPath(const waypoint_follower::lane &waypoints) { current_waypoints_ = waypoints; }
	int getSize() const;
	bool isEmpty() const { return current_waypoints_.waypoints.empty(); };
	double getInterval() const;
	geometry_msgs::Point getWaypointPosition(int waypoint) const;
	geometry_msgs::Quaternion getWaypointOrientation(int waypoint) const;
	geometry_msgs::Pose getWaypointPose(int waypoint) const;
	double getWaypointVelocityMPS(int waypoint) const;
	waypoint_follower::lane getCurrentWaypoints() const { return current_waypoints_; }
  bool isFront(int waypoint, geometry_msgs::Pose current_pose) const;
};

//inline function (less than 10 lines )
inline double kmph2mps(double velocity_kmph) { return (velocity_kmph * 1000) / (60 * 60); }
inline double mps2kmph(double velocity_mps) { return (velocity_mps * 60 * 60) / 1000; }
inline double deg2rad(double deg){  return deg * M_PI/180;} //convert degree to radian


tf::Vector3 point2vector(geometry_msgs::Point point); //convert point to vector
geometry_msgs::Point vector2point(tf::Vector3 vector); //convert vector to point
tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree); //rotate unit vector by degree
geometry_msgs::Point rotatePoint(geometry_msgs::Point point, double degree); //rotate point vector by degree

double DecelerateVelocity(double distance, double prev_velocity);
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point, geometry_msgs::Pose current_pose); //transform point into the coordinate of current_pose
geometry_msgs::Point calcAbsoluteCoordinate(geometry_msgs::Point point, geometry_msgs::Pose current_pose); //transform point into the global coordinate
double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2); //get 2 dimentional distance between target 1 and target 2
int getClosestWaypoint(const waypoint_follower::lane &current_path, geometry_msgs::Pose current_pose);
bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *slope, double *intercept);
double getDistanceBetweenLineAndPoint(geometry_msgs::Point point,double slope,double intercept);
double getRelativeAngle(geometry_msgs::Pose waypoint_pose,geometry_msgs::Pose vehicle_pose);
#endif
