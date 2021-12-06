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

#include <algorithm>
#include <experimental/optional>  // NOLINT
#include <limits>

#include "motion_common/motion_common.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


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
      !isfinite(p.heading_rate_rps))
    {
      return false;
    }
  }

  // when trajectory is empty
  if (traj.points.empty()) {
    return false;
  }

  return true;
}

float64_t calcStopDistance(
  const Point & current_pos, const Trajectory & traj)
{
  const std::experimental::optional<size_t> stop_idx_opt =
    trajectory_common::searchZeroVelocityIndex(traj.points);

  // If no zero velocity point, return the length between current_pose to the end of trajectory.
  if (!stop_idx_opt) {
    return trajectory_common::calcSignedArcLength(traj.points, current_pos, traj.points.size() - 1);
  }

  return trajectory_common::calcSignedArcLength(traj.points, current_pos, *stop_idx_opt);
}

float64_t getPitchByPose(const Quaternion & quaternion_msg)
{
  float64_t roll, pitch, yaw;
  tf2::Quaternion quaternion;
  tf2::fromMsg(quaternion_msg, quaternion);
  tf2::Matrix3x3{quaternion}.getRPY(roll, pitch, yaw);

  return pitch;
}

float64_t getPitchByTraj(
  const Trajectory & trajectory, const size_t nearest_idx, const float64_t wheel_base)
{
  using autoware::common::geometry::distance_2d;
  // cannot calculate pitch
  if (trajectory.points.size() <= 1) {
    return 0.0;
  }

  for (size_t i = nearest_idx + 1; i < trajectory.points.size(); ++i) {
    const float64_t dist =
      distance_2d<float64_t>(trajectory.points.at(nearest_idx), trajectory.points.at(i));
    if (dist > wheel_base) {
      // calculate pitch from trajectory between rear wheel (nearest) and front center (i)
      return calcElevationAngle(
        trajectory.points.at(nearest_idx), trajectory.points.at(i));
    }
  }

  // close to goal
  for (size_t i = trajectory.points.size() - 1; i > 0; --i) {
    const float64_t dist =
      distance_2d<float64_t>(trajectory.points.back(), trajectory.points.at(i));

    if (dist > wheel_base) {
      // calculate pitch from trajectory
      // between wheelbase behind the end of trajectory (i) and the end of trajectory (back)
      return calcElevationAngle(
        trajectory.points.at(i), trajectory.points.back());
    }
  }

  // calculate pitch from trajectory between the beginning and end of trajectory
  return calcElevationAngle(
    trajectory.points.at(0),
    trajectory.points.back());
}

float64_t calcElevationAngle(const TrajectoryPoint & p_from, const TrajectoryPoint & p_to)
{
  const float64_t dx = p_from.pose.position.x - p_to.pose.position.x;
  const float64_t dy = p_from.pose.position.y - p_to.pose.position.y;
  const float64_t dz = p_from.pose.position.z - p_to.pose.position.z;

  const float64_t dxy = std::max(std::hypot(dx, dy), std::numeric_limits<float64_t>::epsilon());
  const float64_t pitch = std::atan2(dz, dxy);

  return pitch;
}

Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const float64_t delay_time, const float64_t current_vel)
{
  // simple linear prediction
  const float64_t yaw = ::motion::motion_common::to_angle(current_pose.orientation);
  const float64_t running_distance = delay_time * current_vel;
  const float64_t dx = running_distance * std::cos(yaw);
  const float64_t dy = running_distance * std::sin(yaw);

  auto pred_pose = current_pose;
  pred_pose.position.x += dx;
  pred_pose.position.y += dy;
  return pred_pose;
}

float64_t lerp(const float64_t v_from, const float64_t v_to, const float64_t ratio)
{
  return v_from + (v_to - v_from) * ratio;
}

Quaternion lerpOrientation(
  const Quaternion & o_from, const Quaternion & o_to, const float64_t ratio)
{
  tf2::Quaternion q_from, q_to;
  tf2::fromMsg(o_from, q_from);
  tf2::fromMsg(o_to, q_to);

  const auto q_interpolated = q_from.slerp(q_to, ratio);
  return tf2::toMsg(q_interpolated);
}

float64_t applyDiffLimitFilter(
  const float64_t input_val, const float64_t prev_val, const float64_t dt, const float64_t max_val,
  const float64_t min_val)
{
  const float64_t diff_raw = (input_val - prev_val) / dt;
  const float64_t diff = std::min(std::max(diff_raw, min_val), max_val);
  const float64_t filtered_val = prev_val + diff * dt;
  return filtered_val;
}

float64_t applyDiffLimitFilter(
  const float64_t input_val, const float64_t prev_val, const float64_t dt, const float64_t lim_val)
{
  const float64_t max_val = std::fabs(lim_val);
  const float64_t min_val = -max_val;
  return applyDiffLimitFilter(input_val, prev_val, dt, max_val, min_val);
}
}  // namespace longitudinal_utils
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
