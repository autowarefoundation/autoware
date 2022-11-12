// Copyright 2021 Tier IV, Inc.
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

#ifndef UTILIZATION__TRAJECTORY_UTILS_HPP_
#define UTILIZATION__TRAJECTORY_UTILS_HPP_

#include <behavior_velocity_planner/planner_data.hpp>
#include <interpolation/linear_interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <motion_velocity_smoother/trajectory_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using geometry_msgs::msg::Quaternion;
using TrajectoryPointWithIdx = std::pair<TrajectoryPoint, size_t>;

inline TrajectoryPoints convertPathToTrajectoryPoints(const PathWithLaneId & path)
{
  TrajectoryPoints tps;
  for (const auto & p : path.points) {
    TrajectoryPoint tp;
    tp.pose = p.point.pose;
    tp.longitudinal_velocity_mps = p.point.longitudinal_velocity_mps;
    // since path point doesn't have acc for now
    tp.acceleration_mps2 = 0;
    tps.emplace_back(tp);
  }
  return tps;
}
inline PathWithLaneId convertTrajectoryPointsToPath(const TrajectoryPoints & trajectory)
{
  PathWithLaneId path;
  for (const auto & p : trajectory) {
    PathPointWithLaneId pp;
    pp.point.pose = p.pose;
    pp.point.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
    path.points.emplace_back(pp);
  }
  return path;
}

inline Quaternion lerpOrientation(
  const Quaternion & o_from, const Quaternion & o_to, const double ratio)
{
  tf2::Quaternion q_from, q_to;
  tf2::fromMsg(o_from, q_from);
  tf2::fromMsg(o_to, q_to);

  const auto q_interpolated = q_from.slerp(q_to, ratio);
  return tf2::toMsg(q_interpolated);
}

//! smooth path point with lane id starts from ego position on path to the path end
inline bool smoothPath(
  const PathWithLaneId & in_path, PathWithLaneId & out_path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  const geometry_msgs::msg::Pose current_pose = planner_data->current_pose.pose;
  const double v0 = planner_data->current_velocity->twist.linear.x;
  const double a0 = planner_data->current_acceleration->accel.accel.linear.x;
  const auto & external_v_limit = planner_data->external_velocity_limit;
  const auto & smoother = planner_data->velocity_smoother_;

  auto trajectory = convertPathToTrajectoryPoints(in_path);
  if (external_v_limit) {
    motion_velocity_smoother::trajectory_utils::applyMaximumVelocityLimit(
      0, trajectory.size(), external_v_limit->max_velocity, trajectory);
  }
  const auto traj_lateral_acc_filtered = smoother->applyLateralAccelerationFilter(trajectory);

  // Resample trajectory with ego-velocity based interval distances
  auto traj_resampled = smoother->resampleTrajectory(
    *traj_lateral_acc_filtered, v0, current_pose, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold);
  const size_t traj_resampled_closest = motion_utils::findFirstNearestIndexWithSoftConstraints(
    traj_resampled, current_pose, planner_data->ego_nearest_dist_threshold,
    planner_data->ego_nearest_yaw_threshold);
  std::vector<TrajectoryPoints> debug_trajectories;
  // Clip trajectory from closest point
  TrajectoryPoints clipped;
  TrajectoryPoints traj_smoothed;
  clipped.insert(
    clipped.end(), traj_resampled.begin() + traj_resampled_closest, traj_resampled.end());
  if (!smoother->apply(v0, a0, clipped, traj_smoothed, debug_trajectories)) {
    std::cerr << "[behavior_velocity][trajectory_utils]: failed to smooth" << std::endl;
    return false;
  }
  traj_smoothed.insert(
    traj_smoothed.begin(), traj_resampled.begin(), traj_resampled.begin() + traj_resampled_closest);

  out_path = convertTrajectoryPointsToPath(traj_smoothed);
  return true;
}

}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__TRAJECTORY_UTILS_HPP_
