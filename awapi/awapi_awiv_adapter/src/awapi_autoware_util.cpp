// Copyright 2020 Tier IV, Inc.
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

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"

#include <limits>

namespace autoware_api
{
double lowpass_filter(const double current_value, const double prev_value, const double gain)
{
  return gain * prev_value + (1.0 - gain) * current_value;
}

namespace planning_util
{
bool calcClosestIndex(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const geometry_msgs::msg::Pose & pose,
  size_t & output_closest_idx, const double dist_thr, const double angle_thr)
{
  double dist_min = std::numeric_limits<double>::max();
  const double yaw_pose = tf2::getYaw(pose.orientation);
  int closest_idx = -1;

  for (int i = 0; i < static_cast<int>(traj.points.size()); ++i) {
    const double dist = calcDist2d(getPose(traj, i).position, pose.position);

    /* check distance threshold */
    if (dist > dist_thr) {
      continue;
    }

    /* check angle threshold */
    double yaw_i = tf2::getYaw(getPose(traj, i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_i);

    if (std::fabs(yaw_diff) > angle_thr) {
      continue;
    }

    if (dist < dist_min) {
      dist_min = dist;
      closest_idx = i;
    }
  }

  output_closest_idx = static_cast<size_t>(closest_idx);

  return closest_idx != -1;
}

double normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI) {
    res -= (2.0 * M_PI);
  }
  while (res < -M_PI) {
    res += 2.0 * M_PI;
  }

  return res;
}

double calcArcLengthFromWayPoint(
  const autoware_auto_planning_msgs::msg::Trajectory & input_path, const size_t src_idx,
  const size_t dst_idx)
{
  double length = 0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    const double dx_wp =
      input_path.points.at(i + 1).pose.position.x - input_path.points.at(i).pose.position.x;
    const double dy_wp =
      input_path.points.at(i + 1).pose.position.y - input_path.points.at(i).pose.position.y;
    length += std::hypot(dx_wp, dy_wp);
  }
  return length;
}

double calcDistanceAlongTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & target_pose)
{
  size_t self_idx;
  size_t stop_idx;
  if (
    !calcClosestIndex(trajectory, current_pose, self_idx) ||
    !calcClosestIndex(trajectory, target_pose, stop_idx)) {
    return std::numeric_limits<double>::max();
  }
  const double dist_to_stop_pose = calcArcLengthFromWayPoint(trajectory, self_idx, stop_idx);
  return dist_to_stop_pose;
}

}  // namespace planning_util

}  // namespace autoware_api
