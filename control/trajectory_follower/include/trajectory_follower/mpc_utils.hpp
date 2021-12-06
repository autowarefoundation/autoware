// Copyright 2018-2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAJECTORY_FOLLOWER__MPC_UTILS_HPP_
#define TRAJECTORY_FOLLOWER__MPC_UTILS_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "trajectory_follower/interpolate.hpp"
#include "trajectory_follower/mpc_trajectory.hpp"
#include "trajectory_follower/visibility_control.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "common/types.hpp"
#include "eigen3/Eigen/Core"
#include "geometry/common_2d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "helper_functions/angle_utils.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/linear_interpolation.hpp"
#include "motion_common/motion_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
namespace MPCUtils
{
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
/**
 * @brief convert from yaw to ros-Quaternion
 * @param [in] yaw input yaw angle
 * @return quaternion
 */
TRAJECTORY_FOLLOWER_PUBLIC geometry_msgs::msg::Quaternion getQuaternionFromYaw(
  const float64_t & yaw);
/**
 * @brief convert Euler angle vector including +-2pi to 0 jump to continuous series data
 * @param [inout] a input angle vector
 */
TRAJECTORY_FOLLOWER_PUBLIC void convertEulerAngleToMonotonic(std::vector<float64_t> * a);
/**
 * @brief calculate the lateral error of the given pose relative to the given reference pose
 * @param [in] ego_pose pose to check for error
 * @param [in] ref_pose reference pose
 * @return lateral distance between the two poses
 */
TRAJECTORY_FOLLOWER_PUBLIC float64_t calcLateralError(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & ref_pose);
/**
 * @brief convert the given Trajectory msg to a MPCTrajectory object
 * @param [in] input trajectory to convert
 * @param [out] output resulting MPCTrajectory
 * @return true if the conversion was successful
 */
TRAJECTORY_FOLLOWER_PUBLIC bool8_t convertToMPCTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & input, MPCTrajectory & output);
/**
 * @brief convert the given MPCTrajectory to a Trajectory msg
 * @param [in] input MPCTrajectory to convert
 * @param [out] output resulting Trajectory msg
 * @return true if the conversion was successful
 */
TRAJECTORY_FOLLOWER_PUBLIC bool8_t convertToAutowareTrajectory(
  const MPCTrajectory & input, autoware_auto_planning_msgs::msg::Trajectory & output);
/**
 * @brief calculate the arc length at each point of the given trajectory
 * @param [in] trajectory trajectory for which to calculate the arc length
 * @param [out] arclength the cummulative arc length at each point of the trajectory
 */
TRAJECTORY_FOLLOWER_PUBLIC void calcMPCTrajectoryArclength(
  const MPCTrajectory & trajectory, std::vector<float64_t> * arclength);
/**
 * @brief resample the given trajectory with the given fixed interval
 * @param [in] input trajectory to resample
 * @param [in] resample_interval_dist the desired distance between two successive trajectory points
 * @param [out] output the resampled trajectory
 */
TRAJECTORY_FOLLOWER_PUBLIC bool8_t resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const float64_t resample_interval_dist, MPCTrajectory * output);
/**
 * @brief linearly interpolate the given trajectory assuming a base indexing and a new desired indexing
 * @param [in] in_index indexes for each trajectory point
 * @param [in] in_traj MPCTrajectory to interpolate
 * @param [in] out_index desired interpolated indexes
 * @param [out] out_traj resulting interpolated MPCTrajectory
 */
TRAJECTORY_FOLLOWER_PUBLIC bool8_t linearInterpMPCTrajectory(
  const std::vector<float64_t> & in_index, const MPCTrajectory & in_traj,
  const std::vector<float64_t> & out_index, MPCTrajectory * out_traj);
/**
 * @brief fill the relative_time field of the given MPCTrajectory
 * @param [in] traj MPCTrajectory for which to fill in the relative_time
 * @return true if the calculation was successful
 */
TRAJECTORY_FOLLOWER_PUBLIC bool8_t calcMPCTrajectoryTime(MPCTrajectory & traj);
/**
 * @brief recalculate the velocity field (vx) of the MPCTrajectory with dynamic smoothing
 * @param [in] start_idx index of the trajectory point from which to start smoothing
 * @param [in] start_vel initial velocity to set at the start_idx
 * @param [in] acc_lim limit on the acceleration
 * @param [in] tau constant to control the smoothing (high-value = very smooth)
 * @param [inout] traj MPCTrajectory for which to calculate the smoothed velocity
 */
TRAJECTORY_FOLLOWER_PUBLIC void dynamicSmoothingVelocity(
  const size_t start_idx, const float64_t start_vel, const float64_t acc_lim, const float64_t tau,
  MPCTrajectory & traj);
/**
 * @brief calculate yaw angle in MPCTrajectory from xy vector
 * @param [inout] traj object trajectory
 * @param [in] nearest_idx trajectory index nearest to ego
 * @param [in] ego_yaw yaw of the ego vehicle
 */
TRAJECTORY_FOLLOWER_PUBLIC void calcTrajectoryYawFromXY(
  MPCTrajectory * traj, const int64_t nearest_idx, const float64_t ego_yaw);
/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when num = 1)
 * @param [in] curvature_smoothing_num_traj index distance for 3 points for curvature calculation
 * @param [in] curvature_smoothing_num_ref_steer index distance for 3 points for smoothed curvature calculation
 * @param [inout] traj object trajectory
 */
TRAJECTORY_FOLLOWER_PUBLIC bool8_t calcTrajectoryCurvature(
  const size_t curvature_smoothing_num_traj,
  const size_t curvature_smoothing_num_ref_steer,
  MPCTrajectory * traj);
/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when num = 1)
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 * @param [in] traj input trajectory
 * @return vector of curvatures at each point of the given trajectory
 */
TRAJECTORY_FOLLOWER_PUBLIC std::vector<float64_t> calcTrajectoryCurvature(
  const size_t curvature_smoothing_num, const MPCTrajectory & traj);
/**
 * @brief calculate nearest pose on MPCTrajectory with linear interpolation
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @param [out] logger to output the reason for failure
 * @param [in] clock to throttle log output
 * @return false when nearest pose couldn't find for some reasons
 */
TRAJECTORY_FOLLOWER_PUBLIC bool8_t calcNearestPoseInterp(
  const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose,
  geometry_msgs::msg::Pose * nearest_pose, size_t * nearest_index, float64_t * nearest_time,
  const rclcpp::Logger & logger, rclcpp::Clock & clock);
/**
 * @brief calculate the index of the trajectory point nearest to the given pose
 * @param [in] traj trajectory to search for the point nearest to the pose
 * @param [in] self_pose pose for which to search the nearest trajectory point
 * @return index of the input trajectory nearest to the pose
 */
TRAJECTORY_FOLLOWER_PUBLIC int64_t calcNearestIndex(
  const MPCTrajectory & traj,
  const geometry_msgs::msg::Pose & self_pose);
/**
 * @brief calculate the index of the trajectory point nearest to the given pose
 * @param [in] traj trajectory to search for the point nearest to the pose
 * @param [in] self_pose pose for which to search the nearest trajectory point
 * @return index of the input trajectory nearest to the pose
 */
TRAJECTORY_FOLLOWER_PUBLIC int64_t calcNearestIndex(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const geometry_msgs::msg::Pose & self_pose);
/**
 * @brief calculate distance to stopped point
 */
TRAJECTORY_FOLLOWER_PUBLIC float64_t calcStopDistance(
  const autoware_auto_planning_msgs::msg::Trajectory & current_trajectory,
  const int64_t origin);
}  // namespace MPCUtils
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__MPC_UTILS_HPP_
