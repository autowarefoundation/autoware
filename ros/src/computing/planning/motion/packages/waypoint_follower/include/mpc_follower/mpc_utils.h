/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#pragma once
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include "autoware_msgs/Lane.h"
#include <autoware_msgs/VehicleStatus.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "mpc_follower/mpc_trajectory.h"

namespace MPCUtils
{

/* Set into [-pi to pi] */
double normalizeAngle(const double a);

/* Continuously connect singularities of Euler angles */
void convertEulerAngleToMonotonic(std::vector<double> &a);

/* Fill vector's component with increasing number */
void fillIncrease(std::vector<double>::iterator first, std::vector<double>::iterator last, double init, double diff);

/* Conversion from yaw to ros-Quaternion */
geometry_msgs::Quaternion getQuaternionFromYaw(const double &yaw);

/* 1D interpolation */
template <typename T1, typename T2>
bool interp1dX(const T1 &index, const T2 &values, const double &ref, double &ret);

bool interp1dMPCTraj(const std::vector<double> &index, const MPCTrajectory &values,
                     const std::vector<double> &ref, MPCTrajectory &ret);

/* calculate path yaw angle from xy vector*/
void calcTrajectoryYawFromXY(MPCTrajectory &traj);

/* Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when "num = 1") */
void calcTrajectoryCurvature(MPCTrajectory &traj, int curvature_smoothing_num);

void convertWaypointsToMPCTraj(const autoware_msgs::Lane &lane, MPCTrajectory &mpc_traj);

/* convert path with resampling */
void convertWaypointsToMPCTrajWithResample(const autoware_msgs::Lane &path, const std::vector<double> &time,
                                           const std::vector<double> &ref_index, const double &d_index, MPCTrajectory &ref_traj);
void convertWaypointsToMPCTrajWithDistanceResample(const autoware_msgs::Lane &path, const std::vector<double> &time,
                                                   const double &dl, MPCTrajectory &ref_traj);
void convertWaypointsToMPCTrajWithTimeResample(const autoware_msgs::Lane &path, const std::vector<double> &time,
                                               const double &dt, MPCTrajectory &ref_traj_);
/* Insert time into path with velocity */
void calcPathRelativeTime(const autoware_msgs::Lane &path, std::vector<double> &path_time);

/* Calculate nearest pose on path */
bool calcNearestPose(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                     unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);

/* Calculate neareset pose on path with interpolation */
bool calcNearestPoseInterp(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                           unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);

class SplineInterpolateXY
{
  bool initialized_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;

public:
  SplineInterpolateXY();
  SplineInterpolateXY(const std::vector<double> &x);
  ~SplineInterpolateXY();
  void generateSpline(const std::vector<double> &x);
  double getValue(const double &s);
  void getValueVector(const std::vector<double> &s_v, std::vector<double> &value_v);
};

}; // namespace MPCUtils