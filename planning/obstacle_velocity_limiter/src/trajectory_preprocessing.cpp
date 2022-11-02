// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_velocity_limiter/trajectory_preprocessing.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace obstacle_velocity_limiter
{

size_t calculateStartIndex(
  const Trajectory & trajectory, const size_t ego_idx, const Float start_distance)
{
  auto dist = 0.0;
  auto idx = ego_idx;
  while (idx + 1 < trajectory.points.size() && dist < start_distance) {
    dist +=
      tier4_autoware_utils::calcDistance2d(trajectory.points[idx], trajectory.points[idx + 1]);
    ++idx;
  }
  return idx;
}

size_t calculateEndIndex(
  const Trajectory & trajectory, const size_t start_idx, const Float max_length,
  const Float max_duration)
{
  auto length = 0.0;
  auto duration = 0.0;
  auto idx = start_idx;
  while (idx + 1 < trajectory.points.size() && length < max_length && duration < max_duration) {
    const auto length_d =
      tier4_autoware_utils::calcDistance2d(trajectory.points[idx], trajectory.points[idx + 1]);
    length += length_d;
    if (trajectory.points[idx].longitudinal_velocity_mps > 0.0)
      duration += length_d / trajectory.points[idx].longitudinal_velocity_mps;
    ++idx;
  }
  return idx;
}

Trajectory downsampleTrajectory(
  const Trajectory & trajectory, const size_t start_idx, const size_t end_idx, const int factor)
{
  if (factor < 1) return trajectory;
  Trajectory downsampled_traj;
  downsampled_traj.header = trajectory.header;
  downsampled_traj.points.reserve((end_idx - start_idx) / factor);
  for (size_t i = start_idx; i <= end_idx; i += factor)
    downsampled_traj.points.push_back(trajectory.points[i]);
  return downsampled_traj;
}

void calculateSteeringAngles(Trajectory & trajectory, const Float wheel_base)
{
  auto t = 0.0;
  auto prev_point = trajectory.points.front();
  auto prev_heading = tf2::getYaw(prev_point.pose.orientation);
  for (auto i = 1ul; i < trajectory.points.size(); ++i) {
    const auto & prev_point = trajectory.points[i - 1];
    auto & point = trajectory.points[i];
    const auto dt = tier4_autoware_utils::calcDistance2d(prev_point, point) /
                    prev_point.longitudinal_velocity_mps;
    t += dt;
    const auto heading = tf2::getYaw(point.pose.orientation);
    const auto d_heading = heading - prev_heading;
    prev_heading = heading;
    point.front_wheel_angle_rad =
      std::atan2(wheel_base * d_heading, point.longitudinal_velocity_mps * dt);
  }
}

Trajectory copyDownsampledVelocity(
  const Trajectory & downsampled_traj, Trajectory trajectory, const size_t start_idx,
  const int factor)
{
  const auto size = std::min(downsampled_traj.points.size(), trajectory.points.size());
  for (size_t i = 0; i < size; ++i) {
    trajectory.points[start_idx + i * factor].longitudinal_velocity_mps =
      downsampled_traj.points[i].longitudinal_velocity_mps;
  }
  return trajectory;
}
}  // namespace obstacle_velocity_limiter
