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

#include "trajectory_follower/mpc_utils.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

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
using autoware::common::geometry::distance_2d;

geometry_msgs::msg::Quaternion getQuaternionFromYaw(const float64_t & yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void convertEulerAngleToMonotonic(std::vector<float64_t> * a)
{
  if (!a) {
    return;
  }
  for (uint64_t i = 1; i < a->size(); ++i) {
    const float64_t da = a->at(i) - a->at(i - 1);
    a->at(i) = a->at(i - 1) + autoware::common::helper_functions::wrap_angle(da);
  }
}

float64_t calcLateralError(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & ref_pose)
{
  const float64_t err_x = ego_pose.position.x - ref_pose.position.x;
  const float64_t err_y = ego_pose.position.y - ref_pose.position.y;
  const float64_t ref_yaw = ::motion::motion_common::to_angle(ref_pose.orientation);
  const float64_t lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

void calcMPCTrajectoryArclength(
  const MPCTrajectory & trajectory, std::vector<float64_t> * arclength)
{
  float64_t dist = 0.0;
  arclength->clear();
  arclength->push_back(dist);
  for (uint64_t i = 1; i < trajectory.size(); ++i) {
    const float64_t dx = trajectory.x.at(i) - trajectory.x.at(i - 1);
    const float64_t dy = trajectory.y.at(i) - trajectory.y.at(i - 1);
    dist += std::sqrt(dx * dx + dy * dy);
    arclength->push_back(dist);
  }
}

bool8_t resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const float64_t resample_interval_dist, MPCTrajectory * output)
{
  if (!output) {
    return false;
  }
  if (input.empty()) {
    *output = input;
    return true;
  }
  std::vector<float64_t> input_arclength;
  calcMPCTrajectoryArclength(input, &input_arclength);

  if (input_arclength.empty()) {
    return false;
  }

  std::vector<float64_t> output_arclength;
  for (float64_t s = 0; s < input_arclength.back(); s += resample_interval_dist) {
    output_arclength.push_back(s);
  }

  std::vector<float64_t> input_yaw = input.yaw;
  convertEulerAngleToMonotonic(&input_yaw);

  output->x = interpolation::slerp(input_arclength, input.x, output_arclength);
  output->y = interpolation::slerp(input_arclength, input.y, output_arclength);
  output->z = interpolation::slerp(input_arclength, input.z, output_arclength);
  output->yaw = interpolation::slerp(input_arclength, input.yaw, output_arclength);
  output->vx = interpolation::lerp(input_arclength, input.vx, output_arclength);
  output->k = interpolation::slerp(input_arclength, input.k, output_arclength);
  output->smooth_k = interpolation::slerp(input_arclength, input.smooth_k, output_arclength);
  output->relative_time = interpolation::lerp(
    input_arclength, input.relative_time,
    output_arclength);

  return true;
}

bool8_t linearInterpMPCTrajectory(
  const std::vector<float64_t> & in_index, const MPCTrajectory & in_traj,
  const std::vector<float64_t> & out_index, MPCTrajectory * out_traj)
{
  if (!out_traj) {
    return false;
  }

  if (in_traj.empty()) {
    *out_traj = in_traj;
    return true;
  }

  std::vector<float64_t> in_traj_yaw = in_traj.yaw;
  convertEulerAngleToMonotonic(&in_traj_yaw);

  if (
    !linearInterpolate(in_index, in_traj.x, out_index, out_traj->x) ||
    !linearInterpolate(in_index, in_traj.y, out_index, out_traj->y) ||
    !linearInterpolate(in_index, in_traj.z, out_index, out_traj->z) ||
    !linearInterpolate(in_index, in_traj_yaw, out_index, out_traj->yaw) ||
    !linearInterpolate(in_index, in_traj.vx, out_index, out_traj->vx) ||
    !linearInterpolate(in_index, in_traj.k, out_index, out_traj->k) ||
    !linearInterpolate(in_index, in_traj.smooth_k, out_index, out_traj->smooth_k) ||
    !linearInterpolate(
      in_index, in_traj.relative_time, out_index, out_traj->relative_time))
  {
    std::cerr << "linearInterpMPCTrajectory error!" << std::endl;
    return false;
  }

  if (out_traj->empty()) {
    std::cerr << "[mpc util] linear interpolation error" << std::endl;
    return false;
  }

  return true;
}

void calcTrajectoryYawFromXY(
  MPCTrajectory * traj, const int64_t nearest_idx,
  const float64_t ego_yaw)
{
  if (traj->yaw.size() < 3) {  // at least 3 points are required to calculate yaw
    return;
  }
  if (traj->yaw.size() != traj->vx.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc_utils"), "trajectory size has no consistency.");
    return;
  }

  // calculate shift direction (forward or backward)
  const int64_t upper_nearest_idx =
    (static_cast<int64_t>(traj->x.size()) - 1 == nearest_idx) ? nearest_idx : nearest_idx + 1;
  const float64_t dx = traj->x[static_cast<size_t>(upper_nearest_idx)] -
    traj->x[static_cast<size_t>(upper_nearest_idx - 1)];
  const float64_t dy = traj->y[static_cast<size_t>(upper_nearest_idx)] -
    traj->y[static_cast<size_t>(upper_nearest_idx - 1)];
  const bool forward_shift =
    std::abs(autoware::common::helper_functions::wrap_angle(std::atan2(dy, dx) - ego_yaw)) <
    M_PI / 2.0;

  // interpolate yaw
  for (int64_t i = 1; i < static_cast<int64_t>(traj->yaw.size()) - 1; ++i) {
    const float64_t dx = traj->x[static_cast<size_t>(i + 1)] - traj->x[static_cast<size_t>(i - 1)];
    const float64_t dy = traj->y[static_cast<size_t>(i + 1)] - traj->y[static_cast<size_t>(i - 1)];
    traj->yaw[static_cast<size_t>(i)] =
      forward_shift ? std::atan2(dy, dx) : std::atan2(dy, dx) + M_PI;
  }
  if (traj->yaw.size() > 1) {
    traj->yaw[0] = traj->yaw[1];
    traj->yaw.back() = traj->yaw[traj->yaw.size() - 2];
  }
}

bool8_t calcTrajectoryCurvature(
  const size_t curvature_smoothing_num_traj,
  const size_t curvature_smoothing_num_ref_steer,
  MPCTrajectory * traj)
{
  if (!traj) {
    return false;
  }

  traj->k = calcTrajectoryCurvature(curvature_smoothing_num_traj, *traj);
  traj->smooth_k = calcTrajectoryCurvature(curvature_smoothing_num_ref_steer, *traj);
  return true;
}

std::vector<float64_t> calcTrajectoryCurvature(
  const size_t curvature_smoothing_num, const MPCTrajectory & traj)
{
  std::vector<float64_t> curvature_vec(traj.x.size());

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::msg::Point p1, p2, p3;
  const size_t max_smoothing_num = static_cast<size_t>(
    std::floor(0.5 * (static_cast<float64_t>(traj.x.size() - 1))));
  const size_t L = std::min(curvature_smoothing_num, max_smoothing_num);
  for (size_t i = L; i < traj.x.size() - L; ++i) {
    const size_t curr_idx = i;
    const size_t prev_idx = curr_idx - L;
    const size_t next_idx = curr_idx + L;
    p1.x = traj.x[prev_idx];
    p2.x = traj.x[curr_idx];
    p3.x = traj.x[next_idx];
    p1.y = traj.y[prev_idx];
    p2.y = traj.y[curr_idx];
    p3.y = traj.y[next_idx];
    const float64_t den = std::max(
      distance_2d<float64_t>(
        p1,
        p2) * distance_2d<float64_t>(p2, p3) * distance_2d<float64_t>(p3, p1),
      std::numeric_limits<float64_t>::epsilon());
    const float64_t curvature =
      2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    curvature_vec.at(curr_idx) = curvature;
  }

  /* first and last curvature is copied from next value */
  for (size_t i = 0; i < std::min(L, traj.x.size()); ++i) {
    curvature_vec.at(i) = curvature_vec.at(std::min(L, traj.x.size() - 1));
    curvature_vec.at(
      traj.x.size() - i -
      1) = curvature_vec.at(std::max(traj.x.size() - L - 1, size_t(0)));
  }
  return curvature_vec;
}

bool8_t convertToMPCTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & input, MPCTrajectory & output)
{
  output.clear();
  for (const autoware_auto_planning_msgs::msg::TrajectoryPoint & p : input.points) {
    const float64_t x = p.pose.position.x;
    const float64_t y = p.pose.position.y;
    const float64_t z = 0.0;
    const float64_t yaw = ::motion::motion_common::to_angle(p.pose.orientation);
    const float64_t vx = p.longitudinal_velocity_mps;
    const float64_t k = 0.0;
    const float64_t t = 0.0;
    output.push_back(x, y, z, yaw, vx, k, k, t);
  }
  calcMPCTrajectoryTime(output);
  return true;
}

bool8_t convertToAutowareTrajectory(
  const MPCTrajectory & input, autoware_auto_planning_msgs::msg::Trajectory & output)
{
  output.points.clear();
  autoware_auto_planning_msgs::msg::TrajectoryPoint p;
  using Real = decltype(p.pose.position.x);
  for (size_t i = 0; i < input.size(); ++i) {
    p.pose.position.x = static_cast<Real>(input.x.at(i));
    p.pose.position.y = static_cast<Real>(input.y.at(i));
    p.pose.position.z = static_cast<Real>(input.z.at(i));
    p.pose.orientation = ::motion::motion_common::from_angle(input.yaw.at(i));
    p.longitudinal_velocity_mps =
      static_cast<decltype(p.longitudinal_velocity_mps)>(input.vx.at(i));
    output.points.push_back(p);
  }
  return true;
}

bool8_t calcMPCTrajectoryTime(MPCTrajectory & traj)
{
  float64_t t = 0.0;
  traj.relative_time.clear();
  traj.relative_time.push_back(t);
  for (size_t i = 0; i < traj.x.size() - 1; ++i) {
    const float64_t dx = traj.x.at(i + 1) - traj.x.at(i);
    const float64_t dy = traj.y.at(i + 1) - traj.y.at(i);
    const float64_t dz = traj.z.at(i + 1) - traj.z.at(i);
    const float64_t dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    const float64_t v = std::max(std::fabs(traj.vx.at(i)), 0.1);
    t += (dist / v);
    traj.relative_time.push_back(t);
  }
  return true;
}

void dynamicSmoothingVelocity(
  const size_t start_idx, const float64_t start_vel, const float64_t acc_lim, const float64_t tau,
  MPCTrajectory & traj)
{
  float64_t curr_v = start_vel;
  traj.vx.at(start_idx) = start_vel;

  for (size_t i = start_idx + 1; i < traj.size(); ++i) {
    const float64_t ds =
      std::hypot(traj.x.at(i) - traj.x.at(i - 1), traj.y.at(i) - traj.y.at(i - 1));
    const float64_t dt = ds /
      std::max(std::fabs(curr_v), std::numeric_limits<float64_t>::epsilon());
    const float64_t a = tau / std::max(tau + dt, std::numeric_limits<float64_t>::epsilon());
    const float64_t updated_v = a * curr_v + (1.0 - a) * traj.vx.at(i);
    const float64_t dv = std::max(-acc_lim * dt, std::min(acc_lim * dt, updated_v - curr_v));
    curr_v = curr_v + dv;
    traj.vx.at(i) = curr_v;
  }
  calcMPCTrajectoryTime(traj);
}

int64_t calcNearestIndex(
  const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose)
{
  if (traj.empty()) {
    return -1;
  }
  const float64_t my_yaw = ::motion::motion_common::to_angle(self_pose.orientation);
  int64_t nearest_idx = -1;
  float64_t min_dist_squared = std::numeric_limits<float64_t>::max();
  for (size_t i = 0; i < traj.size(); ++i) {
    const float64_t dx = self_pose.position.x - traj.x[i];
    const float64_t dy = self_pose.position.y - traj.y[i];
    const float64_t dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const float64_t err_yaw = autoware::common::helper_functions::wrap_angle(my_yaw - traj.yaw[i]);
    if (std::fabs(err_yaw) > (M_PI / 3.0)) {
      continue;
    }
    if (dist_squared < min_dist_squared) {
      min_dist_squared = dist_squared;
      nearest_idx = static_cast<int64_t>(i);
    }
  }
  return nearest_idx;
}

int64_t calcNearestIndex(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const geometry_msgs::msg::Pose & self_pose)
{
  if (traj.points.empty()) {
    return -1;
  }
  const float64_t my_yaw = ::motion::motion_common::to_angle(self_pose.orientation);
  int64_t nearest_idx = -1;
  float64_t min_dist_squared = std::numeric_limits<float64_t>::max();
  for (size_t i = 0; i < traj.points.size(); ++i) {
    const float64_t dx =
      self_pose.position.x - static_cast<float64_t>(traj.points.at(i).pose.position.x);
    const float64_t dy =
      self_pose.position.y - static_cast<float64_t>(traj.points.at(i).pose.position.y);
    const float64_t dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const float64_t traj_yaw =
      ::motion::motion_common::to_angle(traj.points.at(i).pose.orientation);
    const float64_t err_yaw = autoware::common::helper_functions::wrap_angle(my_yaw - traj_yaw);
    if (std::fabs(err_yaw) > (M_PI / 3.0)) {
      continue;
    }
    if (dist_squared < min_dist_squared) {
      min_dist_squared = dist_squared;
      nearest_idx = static_cast<int64_t>(i);
    }
  }
  return nearest_idx;
}

bool8_t calcNearestPoseInterp(
  const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose,
  geometry_msgs::msg::Pose * nearest_pose, size_t * nearest_index, float64_t * nearest_time,
  const rclcpp::Logger & logger, rclcpp::Clock & clock)
{
  if (traj.empty() || !nearest_pose || !nearest_index || !nearest_time) {
    return false;
  }
  const int64_t nearest_idx = calcNearestIndex(traj, self_pose);
  if (nearest_idx == -1) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, clock, 5000, "[calcNearestPoseInterp] fail to get nearest. traj.size = %zu",
      traj.size());
    return false;
  }

  const int64_t traj_size = static_cast<int64_t>(traj.size());

  *nearest_index = static_cast<size_t>(nearest_idx);

  if (traj.size() == 1) {
    nearest_pose->position.x = traj.x[*nearest_index];
    nearest_pose->position.y = traj.y[*nearest_index];
    nearest_pose->orientation = getQuaternionFromYaw(traj.yaw[*nearest_index]);
    *nearest_time = traj.relative_time[*nearest_index];
    return true;
  }

  auto calcSquaredDist =
    [](const geometry_msgs::msg::Pose & p, const MPCTrajectory & t, const size_t idx) {
      const float64_t dx = p.position.x - t.x[idx];
      const float64_t dy = p.position.y - t.y[idx];
      return dx * dx + dy * dy;
    };

  /* get second nearest index = next to nearest_index */
  const size_t next = static_cast<size_t>(std::min(nearest_idx + 1, traj_size - 1));
  const size_t prev = static_cast<size_t>(std::max(nearest_idx - 1, int64_t(0)));
  const float64_t dist_to_next = calcSquaredDist(self_pose, traj, next);
  const float64_t dist_to_prev = calcSquaredDist(self_pose, traj, prev);
  const size_t second_nearest_index = (dist_to_next < dist_to_prev) ? next : prev;

  const float64_t a_sq = calcSquaredDist(self_pose, traj, *nearest_index);
  const float64_t b_sq = calcSquaredDist(self_pose, traj, second_nearest_index);
  const float64_t dx3 = traj.x[*nearest_index] - traj.x[second_nearest_index];
  const float64_t dy3 = traj.y[*nearest_index] - traj.y[second_nearest_index];
  const float64_t c_sq = dx3 * dx3 + dy3 * dy3;

  /* if distance between two points are too close */
  if (c_sq < 1.0E-5) {
    nearest_pose->position.x = traj.x[*nearest_index];
    nearest_pose->position.y = traj.y[*nearest_index];
    nearest_pose->orientation = getQuaternionFromYaw(traj.yaw[*nearest_index]);
    *nearest_time = traj.relative_time[*nearest_index];
    return true;
  }

  /* linear interpolation */
  const float64_t alpha = std::max(std::min(0.5 * (c_sq - a_sq + b_sq) / c_sq, 1.0), 0.0);
  nearest_pose->position.x =
    alpha * traj.x[*nearest_index] + (1 - alpha) * traj.x[second_nearest_index];
  nearest_pose->position.y =
    alpha * traj.y[*nearest_index] + (1 - alpha) * traj.y[second_nearest_index];
  const float64_t tmp_yaw_err =
    autoware::common::helper_functions::wrap_angle(
    traj.yaw[*nearest_index] -
    traj.yaw[second_nearest_index]);
  const float64_t nearest_yaw =
    autoware::common::helper_functions::wrap_angle(
    traj.yaw[second_nearest_index] + alpha * tmp_yaw_err);
  nearest_pose->orientation = getQuaternionFromYaw(nearest_yaw);
  *nearest_time = alpha * traj.relative_time[*nearest_index] +
    (1 - alpha) * traj.relative_time[second_nearest_index];
  return true;
}

float64_t calcStopDistance(
  const autoware_auto_planning_msgs::msg::Trajectory & current_trajectory,
  const int64_t origin)
{
  constexpr float zero_velocity = std::numeric_limits<float>::epsilon();
  const float origin_velocity =
    current_trajectory.points.at(static_cast<size_t>(origin)).longitudinal_velocity_mps;
  float64_t stop_dist = 0.0;

  // search forward
  if (std::fabs(origin_velocity) > zero_velocity) {
    for (int64_t i = origin + 1; i < static_cast<int64_t>(current_trajectory.points.size()) - 1;
      ++i)
    {
      const auto & p0 = current_trajectory.points.at(static_cast<size_t>(i));
      const auto & p1 = current_trajectory.points.at(static_cast<size_t>(i - 1));
      stop_dist += distance_2d<float64_t>(p0, p1);
      if (std::fabs(p0.longitudinal_velocity_mps) < zero_velocity) {
        break;
      }
    }
    return stop_dist;
  }

  // search backward
  for (int64_t i = origin - 1; 0 < i; --i) {
    const auto & p0 = current_trajectory.points.at(static_cast<size_t>(i));
    const auto & p1 = current_trajectory.points.at(static_cast<size_t>(i + 1));
    if (std::fabs(p0.longitudinal_velocity_mps) > zero_velocity) {
      break;
    }
    stop_dist -= distance_2d<float64_t>(p0, p1);
  }
  return stop_dist;
}

}  // namespace MPCUtils
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
