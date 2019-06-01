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
#include <stdio.h>
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
#include <amathutils_lib/amathutils.hpp>

#include "mpc_follower/mpc_trajectory.h"

namespace MPCUtils
{

/**
 * @brief convert eular angle vector including +-2pi to 0 jump to continuous series data 
 * @param [out] a input angle vector
 */
void convertEulerAngleToMonotonic(std::vector<double> &a);


/**
 * @brief interpolate value vector at reference index
 * @param [in] index index vector related to object value (Eigen::Vector or std::vector)
 * @param [in] values object value vector (Eigen::Vector or std::vector)
 * @param [in] ref reference index
 * @param [out] ret interpolated value
 * @return bool to check whether it was interpolated properly  
 */
template <typename T1, typename T2>
bool interp1d(const T1 &index, const T2 &values, const double &ref, double &ret);

/**
 * @brief interpolate MPCTrajectory at index vector
 * @param [in] index index vector related to MPCTrajectory value 
 * @param [in] values MPCTrajectory
 * @param [in] ref reference index
 * @param [out] ret interpolated MPCTrajectory
 * @return bool to check whether it was interpolated properly  
 */
bool interp1dMPCTraj(const std::vector<double> &index, const MPCTrajectory &values,
                     const std::vector<double> &ref, MPCTrajectory &ret);

/**
 * @brief calculate yaw angle in MPCTrajectory from xy vector
 * @param [inout] traj object trajectory
 */
void calcTrajectoryYawFromXY(MPCTrajectory &traj);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when num = 1)
 * @param [inout] traj object trajectory
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 */
void calcTrajectoryCurvature(MPCTrajectory &traj, int curvature_smoothing_num);

/**
 * @brief convert waypoints to MPCTrajectory
 * @param [in] path input waypoints
 * @param [out] mpc_traj converted traj
 */
void convertWaypointsToMPCTraj(const autoware_msgs::Lane &path, MPCTrajectory &mpc_traj);

/**
 * @brief convert waypoints to MPCTraj with interpolation
 * @param [in] path input waypoints 
 * @param [in] path_time waypoints time used for MPCTrajectory relative_time
 * @param [in] ref_index reference index with constant distance
 * @param [in] d_ref_index constant distance of reference index
 * @param [out] ref_traj converted reference trajectory
 */
void convertWaypointsToMPCTrajWithResample(const autoware_msgs::Lane &path, const std::vector<double> &path_time,
                                           const std::vector<double> &ref_index, const double &d_ref_index, MPCTrajectory &ref_traj);

/**
 * @brief convert waypoints to MPCTraj with constant distance interpolation 
 * @param [in] path input waypoints 
 * @param [in] path_time waypoints time used for MPCTrajectory relative_time
 * @param [in] dl distance of interpolated path
 * @param [out] ref_traj converted reference trajectory
 */
void convertWaypointsToMPCTrajWithDistanceResample(const autoware_msgs::Lane &path, const std::vector<double> &path_time,
                                                   const double &dl, MPCTrajectory &ref_traj);

/**
 * @brief convert waypoints to MPCTraj with constant time interpolation 
 * @param [in] path input waypoints 
 * @param [in] path_time waypoints time used for MPCTrajectory relative_time
 * @param [in] dt time span of interpolated path
 * @param [out] ref_traj converted reference trajectory
 */
void convertWaypointsToMPCTrajWithTimeResample(const autoware_msgs::Lane &path, const std::vector<double> &path_time,
                                               const double &dt, MPCTrajectory &ref_traj_);

/**
 * @brief calculate waypoints time with waypoint velocity
 * @param [in] path object waypoints
 * @param [out] path_time calculated waypoints time vector
 */
void calcPathRelativeTime(const autoware_msgs::Lane &path, std::vector<double> &path_time);

/**
 * @brief calculate nearest pose on MPCTrajectory
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose 
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose 
 * @param [out] min_dist_error distance error from nearest pose to self pose
 * @param [out] nearest_yaw_error yaw angle error from nearest pose to self pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @return false when nearest pose couldn't find for some reasons
 */
bool calcNearestPose(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                     unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);

/**
 * @brief calculate nearest pose on MPCTrajectory with linear interpolation
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose 
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose 
 * @param [out] min_dist_error distance error from nearest pose to self pose
 * @param [out] nearest_yaw_error yaw angle error from nearest pose to self pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @return false when nearest pose couldn't find for some reasons
 */
bool calcNearestPoseInterp(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                           unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);

}; // namespace MPCUtils