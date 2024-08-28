// Copyright 2022-2024 TIER IV, Inc.
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

#include "trajectory_preprocessing.hpp"

#include "types.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <optional>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{

size_t calculateStartIndex(
  const TrajectoryPoints & trajectory, const size_t ego_idx, const double start_distance)
{
  auto dist = 0.0;
  auto idx = ego_idx;
  while (idx + 1 < trajectory.size() && dist < start_distance) {
    dist += autoware::universe_utils::calcDistance2d(trajectory[idx], trajectory[idx + 1]);
    ++idx;
  }
  return idx;
}

size_t calculateEndIndex(
  const TrajectoryPoints & trajectory, const size_t start_idx, const double max_length,
  const double max_duration)
{
  auto length = 0.0;
  auto duration = 0.0;
  auto idx = start_idx;
  while (idx + 1 < trajectory.size() && length < max_length && duration < max_duration) {
    const auto length_d =
      autoware::universe_utils::calcDistance2d(trajectory[idx], trajectory[idx + 1]);
    length += length_d;
    if (trajectory[idx].longitudinal_velocity_mps > 0.0)
      duration += length_d / trajectory[idx].longitudinal_velocity_mps;
    ++idx;
  }
  return idx;
}

TrajectoryPoints downsampleTrajectory(
  const TrajectoryPoints & trajectory, const size_t start_idx, const size_t end_idx,
  const int64_t factor)
{
  if (factor < 1) return trajectory;
  TrajectoryPoints downsampled_traj;
  downsampled_traj.reserve((end_idx - start_idx) / factor);
  for (size_t i = start_idx; i <= end_idx; i += factor) downsampled_traj.push_back(trajectory[i]);
  return downsampled_traj;
}

void calculateSteeringAngles(TrajectoryPoints & trajectory, const double wheel_base)
{
  auto prev_point = trajectory.front();
  auto prev_heading = tf2::getYaw(prev_point.pose.orientation);
  for (auto i = 1ul; i < trajectory.size(); ++i) {
    prev_point = trajectory[i - 1];
    auto & point = trajectory[i];
    const auto dt = autoware::universe_utils::calcDistance2d(prev_point, point) /
                    prev_point.longitudinal_velocity_mps;
    const auto heading = tf2::getYaw(point.pose.orientation);
    const auto d_heading = heading - prev_heading;
    prev_heading = heading;
    point.front_wheel_angle_rad =
      static_cast<float>(std::atan2(wheel_base * d_heading, point.longitudinal_velocity_mps * dt));
  }
}

void add_trajectory_point(TrajectoryPoints & trajectory, const geometry_msgs::msg::Point & point)
{
  const auto segment_idx = motion_utils::findNearestSegmentIndex(trajectory, point);
  const auto lat_offset = motion_utils::calcLateralOffset(trajectory, point, segment_idx);
  if (lat_offset < 1.0) {
    const auto lon_offset_to_segment =
      motion_utils::calcLongitudinalOffsetToSegment(trajectory, segment_idx, point);
    const auto inserted_idx =
      motion_utils::insertTargetPoint(segment_idx, lon_offset_to_segment, trajectory);
    if (inserted_idx) {
      trajectory[*inserted_idx].longitudinal_velocity_mps =
        trajectory[segment_idx].longitudinal_velocity_mps;
      trajectory[*inserted_idx].acceleration_mps2 = trajectory[segment_idx].acceleration_mps2;
    }
  }
}
}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
