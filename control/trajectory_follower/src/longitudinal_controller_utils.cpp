// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "trajectory_follower/longitudinal_controller_utils.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <experimental/optional>  // NOLINT

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <algorithm>
#include <limits>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
namespace longitudinal_utils
{

bool isValidTrajectory(const Trajectory & traj)
{
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.pose.position.x) || !isfinite(p.pose.position.y) ||
      !isfinite(p.pose.position.z) || !isfinite(p.pose.orientation.w) ||
      !isfinite(p.pose.orientation.x) || !isfinite(p.pose.orientation.y) ||
      !isfinite(p.pose.orientation.z) || !isfinite(p.longitudinal_velocity_mps) ||
      !isfinite(p.lateral_velocity_mps) || !isfinite(p.acceleration_mps2) ||
      !isfinite(p.heading_rate_rps)) {
      return false;
    }
  }

  // when trajectory is empty
  if (traj.points.empty()) {
    return false;
  }

  return true;
}

double calcStopDistance(
  const Pose & current_pose, const Trajectory & traj, const double max_dist, const double max_yaw)
{
  const auto stop_idx_opt = motion_utils::searchZeroVelocityIndex(traj.points);

  const size_t end_idx = stop_idx_opt ? *stop_idx_opt : traj.points.size() - 1;
  const size_t seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    traj.points, current_pose, max_dist, max_yaw);
  const double signed_length_on_traj = motion_utils::calcSignedArcLength(
    traj.points, current_pose.position, seg_idx, traj.points.at(end_idx).pose.position,
    std::min(end_idx, traj.points.size() - 2));

  if (std::isnan(signed_length_on_traj)) {
    return 0.0;
  }
  return signed_length_on_traj;
}

double getPitchByPose(const Quaternion & quaternion_msg)
{
  double roll, pitch, yaw;
  tf2::Quaternion quaternion;
  tf2::fromMsg(quaternion_msg, quaternion);
  tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);

  return pitch;
}

double getPitchByTraj(
  const Trajectory & trajectory, const size_t nearest_idx, const double wheel_base)
{
  using autoware::common::geometry::distance_2d;
  // cannot calculate pitch
  if (trajectory.points.size() <= 1) {
    return 0.0;
  }

  for (size_t i = nearest_idx + 1; i < trajectory.points.size(); ++i) {
    const double dist =
      distance_2d<double>(trajectory.points.at(nearest_idx), trajectory.points.at(i));
    if (dist > wheel_base) {
      // calculate pitch from trajectory between rear wheel (nearest) and front center (i)
      return calcElevationAngle(trajectory.points.at(nearest_idx), trajectory.points.at(i));
    }
  }

  // close to goal
  for (size_t i = trajectory.points.size() - 1; i > 0; --i) {
    const double dist = distance_2d<double>(trajectory.points.back(), trajectory.points.at(i));

    if (dist > wheel_base) {
      // calculate pitch from trajectory
      // between wheelbase behind the end of trajectory (i) and the end of trajectory (back)
      return calcElevationAngle(trajectory.points.at(i), trajectory.points.back());
    }
  }

  // calculate pitch from trajectory between the beginning and end of trajectory
  return calcElevationAngle(trajectory.points.at(0), trajectory.points.back());
}

double calcElevationAngle(const TrajectoryPoint & p_from, const TrajectoryPoint & p_to)
{
  const double dx = p_from.pose.position.x - p_to.pose.position.x;
  const double dy = p_from.pose.position.y - p_to.pose.position.y;
  const double dz = p_from.pose.position.z - p_to.pose.position.z;

  const double dxy = std::max(std::hypot(dx, dy), std::numeric_limits<double>::epsilon());
  const double pitch = std::atan2(dz, dxy);

  return pitch;
}

Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel)
{
  // simple linear prediction
  const double yaw = tf2::getYaw(current_pose.orientation);
  const double running_distance = delay_time * current_vel;
  const double dx = running_distance * std::cos(yaw);
  const double dy = running_distance * std::sin(yaw);

  auto pred_pose = current_pose;
  pred_pose.position.x += dx;
  pred_pose.position.y += dy;
  return pred_pose;
}

double lerp(const double v_from, const double v_to, const double ratio)
{
  return v_from + (v_to - v_from) * ratio;
}

Quaternion lerpOrientation(const Quaternion & o_from, const Quaternion & o_to, const double ratio)
{
  tf2::Quaternion q_from, q_to;
  tf2::fromMsg(o_from, q_from);
  tf2::fromMsg(o_to, q_to);

  const auto q_interpolated = q_from.slerp(q_to, ratio);
  return tf2::toMsg(q_interpolated);
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double max_val,
  const double min_val)
{
  const double diff_raw = (input_val - prev_val) / dt;
  const double diff = std::min(std::max(diff_raw, min_val), max_val);
  const double filtered_val = prev_val + diff * dt;
  return filtered_val;
}

double applyDiffLimitFilter(
  const double input_val, const double prev_val, const double dt, const double lim_val)
{
  const double max_val = std::fabs(lim_val);
  const double min_val = -max_val;
  return applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val);
}
}  // namespace longitudinal_utils
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
