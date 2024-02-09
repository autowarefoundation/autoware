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

#include "pid_longitudinal_controller/longitudinal_controller_utils.hpp"

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

namespace autoware::motion::control::pid_longitudinal_controller
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
  const Trajectory & trajectory, const size_t start_idx, const double wheel_base)
{
  // cannot calculate pitch
  if (trajectory.points.size() <= 1) {
    return 0.0;
  }

  const auto [prev_idx, next_idx] = [&]() {
    for (size_t i = start_idx + 1; i < trajectory.points.size(); ++i) {
      const double dist = tier4_autoware_utils::calcDistance3d(
        trajectory.points.at(start_idx), trajectory.points.at(i));
      if (dist > wheel_base) {
        // calculate pitch from trajectory between rear wheel (nearest) and front center (i)
        return std::make_pair(start_idx, i);
      }
    }
    // NOTE: The ego pose is close to the goal.
    return std::make_pair(
      std::min(start_idx, trajectory.points.size() - 2), trajectory.points.size() - 1);
  }();

  return tier4_autoware_utils::calcElevationAngle(
    trajectory.points.at(prev_idx).pose.position, trajectory.points.at(next_idx).pose.position);
}

Pose calcPoseAfterTimeDelay(
  const Pose & current_pose, const double delay_time, const double current_vel,
  const double current_acc)
{
  if (delay_time <= 0.0) {
    return current_pose;
  }

  // check time to stop
  const double time_to_stop = -current_vel / current_acc;

  const double delay_time_calculation =
    time_to_stop > 0.0 && time_to_stop < delay_time ? time_to_stop : delay_time;
  // simple linear prediction
  const double yaw = tf2::getYaw(current_pose.orientation);
  const double running_distance = delay_time_calculation * current_vel + 0.5 * current_acc *
                                                                           delay_time_calculation *
                                                                           delay_time_calculation;
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

geometry_msgs::msg::Pose findTrajectoryPoseAfterDistance(
  const size_t src_idx, const double distance,
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory)
{
  double remain_dist = distance;
  geometry_msgs::msg::Pose p = trajectory.points.back().pose;
  for (size_t i = src_idx; i < trajectory.points.size() - 1; ++i) {
    const double dist = tier4_autoware_utils::calcDistance3d(
      trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose);
    if (remain_dist < dist) {
      if (remain_dist <= 0.0) {
        return trajectory.points.at(i).pose;
      }
      double ratio = remain_dist / dist;
      const auto p0 = trajectory.points.at(i).pose;
      const auto p1 = trajectory.points.at(i + 1).pose;
      p = trajectory.points.at(i).pose;
      p.position.x = interpolation::lerp(p0.position.x, p1.position.x, ratio);
      p.position.y = interpolation::lerp(p0.position.y, p1.position.y, ratio);
      p.position.z = interpolation::lerp(p0.position.z, p1.position.z, ratio);
      p.orientation = interpolation::lerpOrientation(p0.orientation, p1.orientation, ratio);
      break;
    }
    remain_dist -= dist;
  }
  return p;
}
}  // namespace longitudinal_utils
}  // namespace autoware::motion::control::pid_longitudinal_controller
