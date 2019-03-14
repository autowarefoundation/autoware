#pragma once
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Core>
#include "autoware_msgs/Lane.h"
#include <autoware_msgs/VehicleStatus.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "mpc_follower/mpc_trajectory.h"

namespace MPCUtils
{

/* Set into [-pi to pi] */
double intoSemicircle(const double a);

/* Continuously connect singularities of Euler angles */
void convertEulerAngleToMonotonic(std::vector<double> &a);

/* Fill vector's component with increasing number */
void fillIncrease(std::vector<double>::iterator first, std::vector<double>::iterator last, double init, double diff);

/* Conversion from yaw to ros-Quaternion */
geometry_msgs::Quaternion getQuaternionFromYaw(const double &yaw);

/* 1D interpolation */
bool interp1d(const std::vector<double> &index, const std::vector<double> &values, const double &ref, double &ret);
bool interp1d(const Eigen::VectorXd &index, const Eigen::VectorXd &values, const double &ref, double &ret);
bool interp1dMPCTraj(const std::vector<double> &index, const MPCTrajectory &values,
                     const std::vector<double> &ref, MPCTrajectory &ret);

template <typename T1, typename T2>
bool interp1dX(const T1 &index, const T2 &values, const double &ref, double &ret);

/* calculate path yaw angle from xy vector*/
void calcTrajectoryYawFromXY(MPCTrajectory &traj);

/* Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when "num = 1") */
void calcTrajectoryCurvature(MPCTrajectory &traj, int curvature_smoothing_num);

/* Resample path with constant distance */
void resamplePathToTrajByDistance(const autoware_msgs::Lane &path, const std::vector<double> &time,
                                  const double &dl, MPCTrajectory &ref_traj);

/* Resample path with constant time */
void resamplePathToTrajByTime(const autoware_msgs::Lane &path, const std::vector<double> &time,
                              const double &dt, MPCTrajectory &ref_traj_);
/* Insert time into path with velocity */
void calcPathRelativeTime(const autoware_msgs::Lane &path, std::vector<double> &path_time);

/* Calculate nearest pose on path */
void calcNearestPose(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                     unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);

/* Calculate neareset pose on path with interpolation */
void calcNearestPoseInterp(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                           unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);
}; // namespace MPCUtils