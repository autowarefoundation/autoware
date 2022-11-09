// Copyright 2018-2021 The Autoware Foundation
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

#include "trajectory_follower/mpc_utils.hpp"

#include "motion_utils/motion_utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

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

geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double & yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void convertEulerAngleToMonotonic(std::vector<double> * a)
{
  if (!a) {
    return;
  }
  for (uint i = 1; i < a->size(); ++i) {
    const double da = a->at(i) - a->at(i - 1);
    a->at(i) = a->at(i - 1) + autoware::common::helper_functions::wrap_angle(da);
  }
}

double calcLateralError(
  const geometry_msgs::msg::Pose & ego_pose, const geometry_msgs::msg::Pose & ref_pose)
{
  const double err_x = ego_pose.position.x - ref_pose.position.x;
  const double err_y = ego_pose.position.y - ref_pose.position.y;
  const double ref_yaw = tf2::getYaw(ref_pose.orientation);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

void calcMPCTrajectoryArclength(const MPCTrajectory & trajectory, std::vector<double> * arclength)
{
  double dist = 0.0;
  arclength->clear();
  arclength->push_back(dist);
  for (uint i = 1; i < trajectory.size(); ++i) {
    const double dx = trajectory.x.at(i) - trajectory.x.at(i - 1);
    const double dy = trajectory.y.at(i) - trajectory.y.at(i - 1);
    dist += std::sqrt(dx * dx + dy * dy);
    arclength->push_back(dist);
  }
}

bool resampleMPCTrajectoryByDistance(
  const MPCTrajectory & input, const double resample_interval_dist, MPCTrajectory * output)
{
  if (!output) {
    return false;
  }
  if (input.empty()) {
    *output = input;
    return true;
  }
  std::vector<double> input_arclength;
  calcMPCTrajectoryArclength(input, &input_arclength);

  if (input_arclength.empty()) {
    return false;
  }

  std::vector<double> output_arclength;
  for (double s = 0; s < input_arclength.back(); s += resample_interval_dist) {
    output_arclength.push_back(s);
  }

  std::vector<double> input_yaw = input.yaw;
  convertEulerAngleToMonotonic(&input_yaw);

  output->x = interpolation::spline(input_arclength, input.x, output_arclength);
  output->y = interpolation::spline(input_arclength, input.y, output_arclength);
  output->z = interpolation::spline(input_arclength, input.z, output_arclength);
  output->yaw = interpolation::spline(input_arclength, input.yaw, output_arclength);
  output->vx = interpolation::lerp(input_arclength, input.vx, output_arclength);
  output->k = interpolation::spline(input_arclength, input.k, output_arclength);
  output->smooth_k = interpolation::spline(input_arclength, input.smooth_k, output_arclength);
  output->relative_time =
    interpolation::lerp(input_arclength, input.relative_time, output_arclength);

  return true;
}

bool linearInterpMPCTrajectory(
  const std::vector<double> & in_index, const MPCTrajectory & in_traj,
  const std::vector<double> & out_index, MPCTrajectory * out_traj)
{
  if (!out_traj) {
    return false;
  }

  if (in_traj.empty()) {
    *out_traj = in_traj;
    return true;
  }

  std::vector<double> in_traj_yaw = in_traj.yaw;
  convertEulerAngleToMonotonic(&in_traj_yaw);

  if (
    !linearInterpolate(in_index, in_traj.x, out_index, out_traj->x) ||
    !linearInterpolate(in_index, in_traj.y, out_index, out_traj->y) ||
    !linearInterpolate(in_index, in_traj.z, out_index, out_traj->z) ||
    !linearInterpolate(in_index, in_traj_yaw, out_index, out_traj->yaw) ||
    !linearInterpolate(in_index, in_traj.vx, out_index, out_traj->vx) ||
    !linearInterpolate(in_index, in_traj.k, out_index, out_traj->k) ||
    !linearInterpolate(in_index, in_traj.smooth_k, out_index, out_traj->smooth_k) ||
    !linearInterpolate(in_index, in_traj.relative_time, out_index, out_traj->relative_time)) {
    std::cerr << "linearInterpMPCTrajectory error!" << std::endl;
    return false;
  }

  if (out_traj->empty()) {
    std::cerr << "[mpc util] linear interpolation error" << std::endl;
    return false;
  }

  return true;
}

void calcTrajectoryYawFromXY(MPCTrajectory * traj, const bool is_forward_shift)
{
  if (traj->yaw.size() < 3) {  // at least 3 points are required to calculate yaw
    return;
  }
  if (traj->yaw.size() != traj->vx.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("mpc_utils"), "trajectory size has no consistency.");
    return;
  }

  // interpolate yaw
  for (int i = 1; i < static_cast<int>(traj->yaw.size()) - 1; ++i) {
    const double dx = traj->x[static_cast<size_t>(i + 1)] - traj->x[static_cast<size_t>(i - 1)];
    const double dy = traj->y[static_cast<size_t>(i + 1)] - traj->y[static_cast<size_t>(i - 1)];
    traj->yaw[static_cast<size_t>(i)] =
      is_forward_shift ? std::atan2(dy, dx) : std::atan2(dy, dx) + M_PI;
  }
  if (traj->yaw.size() > 1) {
    traj->yaw[0] = traj->yaw[1];
    traj->yaw.back() = traj->yaw[traj->yaw.size() - 2];
  }
}

bool calcTrajectoryCurvature(
  const size_t curvature_smoothing_num_traj, const size_t curvature_smoothing_num_ref_steer,
  MPCTrajectory * traj)
{
  if (!traj) {
    return false;
  }

  traj->k = calcTrajectoryCurvature(curvature_smoothing_num_traj, *traj);
  traj->smooth_k = calcTrajectoryCurvature(curvature_smoothing_num_ref_steer, *traj);
  return true;
}

std::vector<double> calcTrajectoryCurvature(
  const size_t curvature_smoothing_num, const MPCTrajectory & traj)
{
  std::vector<double> curvature_vec(traj.x.size());

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::msg::Point p1, p2, p3;
  const size_t max_smoothing_num =
    static_cast<size_t>(std::floor(0.5 * (static_cast<double>(traj.x.size() - 1))));
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
    const double den = std::max(
      distance_2d<double>(p1, p2) * distance_2d<double>(p2, p3) * distance_2d<double>(p3, p1),
      std::numeric_limits<double>::epsilon());
    const double curvature =
      2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    curvature_vec.at(curr_idx) = curvature;
  }

  /* first and last curvature is copied from next value */
  for (size_t i = 0; i < std::min(L, traj.x.size()); ++i) {
    curvature_vec.at(i) = curvature_vec.at(std::min(L, traj.x.size() - 1));
    curvature_vec.at(traj.x.size() - i - 1) =
      curvature_vec.at(std::max(traj.x.size() - L - 1, size_t(0)));
  }
  return curvature_vec;
}

bool convertToMPCTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & input, MPCTrajectory & output)
{
  output.clear();
  for (const autoware_auto_planning_msgs::msg::TrajectoryPoint & p : input.points) {
    const double x = p.pose.position.x;
    const double y = p.pose.position.y;
    const double z = 0.0;
    const double yaw = tf2::getYaw(p.pose.orientation);
    const double vx = p.longitudinal_velocity_mps;
    const double k = 0.0;
    const double t = 0.0;
    output.push_back(x, y, z, yaw, vx, k, k, t);
  }
  calcMPCTrajectoryTime(output);
  return true;
}

bool convertToAutowareTrajectory(
  const MPCTrajectory & input, autoware_auto_planning_msgs::msg::Trajectory & output)
{
  output.points.clear();
  autoware_auto_planning_msgs::msg::TrajectoryPoint p;
  using Real = decltype(p.pose.position.x);
  for (size_t i = 0; i < input.size(); ++i) {
    p.pose.position.x = static_cast<Real>(input.x.at(i));
    p.pose.position.y = static_cast<Real>(input.y.at(i));
    p.pose.position.z = static_cast<Real>(input.z.at(i));
    p.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(input.yaw.at(i));
    p.longitudinal_velocity_mps =
      static_cast<decltype(p.longitudinal_velocity_mps)>(input.vx.at(i));
    output.points.push_back(p);
  }
  return true;
}

bool calcMPCTrajectoryTime(MPCTrajectory & traj)
{
  double t = 0.0;
  traj.relative_time.clear();
  traj.relative_time.push_back(t);
  for (size_t i = 0; i < traj.x.size() - 1; ++i) {
    const double dx = traj.x.at(i + 1) - traj.x.at(i);
    const double dy = traj.y.at(i + 1) - traj.y.at(i);
    const double dz = traj.z.at(i + 1) - traj.z.at(i);
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double v = std::max(std::fabs(traj.vx.at(i)), 0.1);
    t += (dist / v);
    traj.relative_time.push_back(t);
  }
  return true;
}

void dynamicSmoothingVelocity(
  const size_t start_idx, const double start_vel, const double acc_lim, const double tau,
  MPCTrajectory & traj)
{
  double curr_v = start_vel;
  traj.vx.at(start_idx) = start_vel;

  for (size_t i = start_idx + 1; i < traj.size(); ++i) {
    const double ds = std::hypot(traj.x.at(i) - traj.x.at(i - 1), traj.y.at(i) - traj.y.at(i - 1));
    const double dt = ds / std::max(std::fabs(curr_v), std::numeric_limits<double>::epsilon());
    const double a = tau / std::max(tau + dt, std::numeric_limits<double>::epsilon());
    const double updated_v = a * curr_v + (1.0 - a) * traj.vx.at(i);
    const double dv = std::max(-acc_lim * dt, std::min(acc_lim * dt, updated_v - curr_v));
    curr_v = curr_v + dv;
    traj.vx.at(i) = curr_v;
  }
  calcMPCTrajectoryTime(traj);
}

bool calcNearestPoseInterp(
  const MPCTrajectory & traj, const geometry_msgs::msg::Pose & self_pose,
  geometry_msgs::msg::Pose * nearest_pose, size_t * nearest_index, double * nearest_time,
  const double max_dist, const double max_yaw, const rclcpp::Logger & logger, rclcpp::Clock & clock)
{
  if (traj.empty() || !nearest_pose || !nearest_index || !nearest_time) {
    return false;
  }

  autoware_auto_planning_msgs::msg::Trajectory autoware_traj;
  convertToAutowareTrajectory(traj, autoware_traj);
  if (autoware_traj.points.empty()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger, clock, 5000, "[calcNearestPoseInterp] input trajectory is empty");
    return false;
  }

  *nearest_index = motion_utils::findFirstNearestIndexWithSoftConstraints(
    autoware_traj.points, self_pose, max_dist, max_yaw);
  const size_t traj_size = traj.size();

  if (traj.size() == 1) {
    nearest_pose->position.x = traj.x[*nearest_index];
    nearest_pose->position.y = traj.y[*nearest_index];
    nearest_pose->orientation = getQuaternionFromYaw(traj.yaw[*nearest_index]);
    *nearest_time = traj.relative_time[*nearest_index];
    return true;
  }

  auto calcSquaredDist =
    [](const geometry_msgs::msg::Pose & p, const MPCTrajectory & t, const size_t idx) {
      const double dx = p.position.x - t.x[idx];
      const double dy = p.position.y - t.y[idx];
      return dx * dx + dy * dy;
    };

  /* get second nearest index = next to nearest_index */
  const size_t next = static_cast<size_t>(
    std::min(static_cast<int>(*nearest_index) + 1, static_cast<int>(traj_size) - 1));
  const size_t prev =
    static_cast<size_t>(std::max(static_cast<int>(*nearest_index) - 1, static_cast<int>(0)));
  const double dist_to_next = calcSquaredDist(self_pose, traj, next);
  const double dist_to_prev = calcSquaredDist(self_pose, traj, prev);
  const size_t second_nearest_index = (dist_to_next < dist_to_prev) ? next : prev;

  const double a_sq = calcSquaredDist(self_pose, traj, *nearest_index);
  const double b_sq = calcSquaredDist(self_pose, traj, second_nearest_index);
  const double dx3 = traj.x[*nearest_index] - traj.x[second_nearest_index];
  const double dy3 = traj.y[*nearest_index] - traj.y[second_nearest_index];
  const double c_sq = dx3 * dx3 + dy3 * dy3;

  /* if distance between two points are too close */
  if (c_sq < 1.0E-5) {
    nearest_pose->position.x = traj.x[*nearest_index];
    nearest_pose->position.y = traj.y[*nearest_index];
    nearest_pose->orientation = getQuaternionFromYaw(traj.yaw[*nearest_index]);
    *nearest_time = traj.relative_time[*nearest_index];
    return true;
  }

  /* linear interpolation */
  const double alpha = std::max(std::min(0.5 * (c_sq - a_sq + b_sq) / c_sq, 1.0), 0.0);
  nearest_pose->position.x =
    alpha * traj.x[*nearest_index] + (1 - alpha) * traj.x[second_nearest_index];
  nearest_pose->position.y =
    alpha * traj.y[*nearest_index] + (1 - alpha) * traj.y[second_nearest_index];
  const double tmp_yaw_err = autoware::common::helper_functions::wrap_angle(
    traj.yaw[*nearest_index] - traj.yaw[second_nearest_index]);
  const double nearest_yaw = autoware::common::helper_functions::wrap_angle(
    traj.yaw[second_nearest_index] + alpha * tmp_yaw_err);
  nearest_pose->orientation = getQuaternionFromYaw(nearest_yaw);
  *nearest_time = alpha * traj.relative_time[*nearest_index] +
                  (1 - alpha) * traj.relative_time[second_nearest_index];
  return true;
}

double calcStopDistance(
  const autoware_auto_planning_msgs::msg::Trajectory & current_trajectory, const int origin)
{
  constexpr float zero_velocity = std::numeric_limits<float>::epsilon();
  const float origin_velocity =
    current_trajectory.points.at(static_cast<size_t>(origin)).longitudinal_velocity_mps;
  double stop_dist = 0.0;

  // search forward
  if (std::fabs(origin_velocity) > zero_velocity) {
    for (int i = origin + 1; i < static_cast<int>(current_trajectory.points.size()) - 1; ++i) {
      const auto & p0 = current_trajectory.points.at(static_cast<size_t>(i));
      const auto & p1 = current_trajectory.points.at(static_cast<size_t>(i - 1));
      stop_dist += distance_2d<double>(p0, p1);
      if (std::fabs(p0.longitudinal_velocity_mps) < zero_velocity) {
        break;
      }
    }
    return stop_dist;
  }

  // search backward
  for (int i = origin - 1; 0 < i; --i) {
    const auto & p0 = current_trajectory.points.at(static_cast<size_t>(i));
    const auto & p1 = current_trajectory.points.at(static_cast<size_t>(i + 1));
    if (std::fabs(p0.longitudinal_velocity_mps) > zero_velocity) {
      break;
    }
    stop_dist -= distance_2d<double>(p0, p1);
  }
  return stop_dist;
}

}  // namespace MPCUtils
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
