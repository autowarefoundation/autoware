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

#include "motion_velocity_smoother/trajectory_utils.hpp"

#include "interpolation/linear_interpolation.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace motion_velocity_smoother
{
using geometry_msgs::msg::Point;
namespace trajectory_utils
{
inline void convertEulerAngleToMonotonic(std::vector<double> & a)
{
  for (unsigned int i = 1; i < a.size(); ++i) {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + tier4_autoware_utils::normalizeRadian(da);
  }
}

inline tf2::Vector3 getTransVector3(const Pose & from, const Pose & to)
{
  double dx = to.position.x - from.position.x;
  double dy = to.position.y - from.position.y;
  double dz = to.position.z - from.position.z;
  return tf2::Vector3(dx, dy, dz);
}

inline double integ_x(double x0, double v0, double a0, double j0, double t)
{
  return x0 + v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * j0 * t * t * t;
}

inline double integ_v(double v0, double a0, double j0, double t)
{
  return v0 + a0 * t + 0.5 * j0 * t * t;
}

inline double integ_a(double a0, double j0, double t)
{
  return a0 + j0 * t;
}

TrajectoryPoint calcInterpolatedTrajectoryPoint(
  const TrajectoryPoints & trajectory, const Pose & target_pose, const size_t seg_idx)
{
  TrajectoryPoint traj_p{};
  traj_p.pose = target_pose;

  if (trajectory.empty()) {
    traj_p.longitudinal_velocity_mps = 0.0;
    traj_p.acceleration_mps2 = 0.0;
    return traj_p;
  }

  if (trajectory.size() == 1) {
    traj_p.longitudinal_velocity_mps = trajectory.at(0).longitudinal_velocity_mps;
    traj_p.acceleration_mps2 = trajectory.at(0).acceleration_mps2;
    return traj_p;
  }

  auto v1 = getTransVector3(trajectory.at(seg_idx).pose, trajectory.at(seg_idx + 1).pose);
  auto v2 = getTransVector3(trajectory.at(seg_idx).pose, target_pose);
  // calc internal proportion
  const double prop{std::max(0.0, std::min(1.0, v1.dot(v2) / v1.length2()))};

  {
    const auto & seg_pt = trajectory.at(seg_idx);
    const auto & next_pt = trajectory.at(seg_idx + 1);
    traj_p.pose = tier4_autoware_utils::calcInterpolatedPose(seg_pt.pose, next_pt.pose, prop);
    traj_p.longitudinal_velocity_mps = interpolation::lerp(
      seg_pt.longitudinal_velocity_mps, next_pt.longitudinal_velocity_mps, prop);
    traj_p.acceleration_mps2 =
      interpolation::lerp(seg_pt.acceleration_mps2, next_pt.acceleration_mps2, prop);
  }

  return traj_p;
}

TrajectoryPoints extractPathAroundIndex(
  const TrajectoryPoints & trajectory, const size_t index, const double & ahead_length,
  const double & behind_length)
{
  if (trajectory.size() == 0 || trajectory.size() - 1 < index) {
    return {};
  }

  // calc ahead distance
  size_t ahead_index{trajectory.size() - 1};
  {
    double dist_sum = 0.0;
    for (size_t i = index; i < trajectory.size() - 1; ++i) {
      dist_sum += tier4_autoware_utils::calcDistance2d(trajectory.at(i), trajectory.at(i + 1));
      if (dist_sum > ahead_length) {
        ahead_index = i + 1;
        break;
      }
    }
  }

  // calc behind distance
  size_t behind_index{0};
  {
    double dist_sum{0.0};
    for (size_t i = index; i != 0; --i) {
      dist_sum += tier4_autoware_utils::calcDistance2d(trajectory.at(i), trajectory[i - 1]);
      if (dist_sum > behind_length) {
        behind_index = i - 1;
        break;
      }
    }
  }

  // extract trajectory
  TrajectoryPoints extracted_traj{};
  for (size_t i = behind_index; i < ahead_index + 1; ++i) {
    extracted_traj.push_back(trajectory.at(i));
  }

  return extracted_traj;
}

std::vector<double> calcArclengthArray(const TrajectoryPoints & trajectory)
{
  if (trajectory.empty()) {
    return {};
  }

  std::vector<double> arclength(trajectory.size());
  double dist = 0.0;
  arclength.front() = dist;
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    const TrajectoryPoint tp = trajectory.at(i);
    const TrajectoryPoint tp_prev = trajectory.at(i - 1);
    dist += tier4_autoware_utils::calcDistance2d(tp.pose, tp_prev.pose);
    arclength.at(i) = dist;
  }
  return arclength;
}

std::vector<double> calcTrajectoryIntervalDistance(const TrajectoryPoints & trajectory)
{
  std::vector<double> intervals;
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    const TrajectoryPoint tp = trajectory.at(i);
    const TrajectoryPoint tp_prev = trajectory.at(i - 1);
    const double dist = tier4_autoware_utils::calcDistance2d(tp.pose, tp_prev.pose);
    intervals.push_back(dist);
  }
  return intervals;
}

std::vector<double> calcTrajectoryCurvatureFrom3Points(
  const TrajectoryPoints & trajectory, size_t idx_dist)
{
  using tier4_autoware_utils::calcCurvature;
  using tier4_autoware_utils::getPoint;

  if (trajectory.size() < 3) {
    const std::vector<double> k_arr(trajectory.size(), 0.0);
    return k_arr;
  }

  // if the idx size is not enough, change the idx_dist
  const auto max_idx_dist = static_cast<size_t>(std::floor((trajectory.size() - 1) / 2.0));
  idx_dist = std::max(1ul, std::min(idx_dist, max_idx_dist));

  if (idx_dist < 1) {
    throw std::logic_error("idx_dist less than 1 is not expected");
  }

  // calculate curvature by circle fitting from three points
  std::vector<double> k_arr(trajectory.size(), 0.0);

  for (size_t i = 1; i + 1 < trajectory.size(); i++) {
    double curvature = 0.0;
    const auto p0 = getPoint(trajectory.at(i - std::min(idx_dist, i)));
    const auto p1 = getPoint(trajectory.at(i));
    const auto p2 = getPoint(trajectory.at(i + std::min(idx_dist, trajectory.size() - 1 - i)));
    try {
      curvature = calcCurvature(p0, p1, p2);
    } catch (std::exception const & e) {
      // ...code that handles the error...
      RCLCPP_WARN(
        rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"), "%s",
        e.what());
      if (i > 1) {
        curvature = k_arr.at(i - 1);  // previous curvature
      } else {
        curvature = 0.0;
      }
    }
    k_arr.at(i) = curvature;
  }
  // copy curvatures for the last and first points;
  k_arr.at(0) = k_arr.at(1);
  k_arr.back() = k_arr.at((trajectory.size() - 2));

  return k_arr;
}

void setZeroVelocity(TrajectoryPoints & trajectory)
{
  for (auto & tp : trajectory) {
    tp.longitudinal_velocity_mps = 0.0;
  }
}

double getMaxVelocity(const TrajectoryPoints & trajectory)
{
  double max_vel = 0.0;
  for (auto & tp : trajectory) {
    if (tp.longitudinal_velocity_mps > max_vel) {
      max_vel = tp.longitudinal_velocity_mps;
    }
  }
  return max_vel;
}

double getMaxAbsVelocity(const TrajectoryPoints & trajectory)
{
  double max_vel = 0.0;
  for (auto & tp : trajectory) {
    double abs_vel = std::fabs(tp.longitudinal_velocity_mps);
    if (abs_vel > max_vel) {
      max_vel = abs_vel;
    }
  }
  return max_vel;
}

void applyMaximumVelocityLimit(
  const size_t begin, const size_t end, const double max_vel, TrajectoryPoints & trajectory)
{
  for (size_t idx = begin; idx < end; ++idx) {
    if (trajectory.at(idx).longitudinal_velocity_mps > max_vel) {
      trajectory.at(idx).longitudinal_velocity_mps = max_vel;
    }
  }
}

bool calcStopDistWithJerkConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, std::map<double, double> & jerk_profile,
  double & stop_dist)
{
  auto calculateTime = [](const double a0, const double target_acc, const double jerk) {
    return (target_acc - a0) / jerk;
  };
  enum class AccelerationType { TRAPEZOID = 0, TRIANGLE = 1, LINEAR = 2 } type;

  jerk_profile.clear();

  constexpr double t_threshold = 1e-06;  // Threshold for truncating value to 0
  constexpr double a_target = 0.0;       // [m/s^2]

  // Calculate time interval with constant acceleration (min_acc)
  double t_min;
  {
    const double v_diff_jerk_dec = 0.5 * calculateTime(a0, 0.0, jerk_dec) * a0 +
                                   0.5 * calculateTime(0.0, min_acc, jerk_dec) * min_acc;
    const double v_diff_jerk_acc = 0.5 * calculateTime(min_acc, 0.0, jerk_acc) * min_acc;
    const double v_error = target_vel - v0;
    // From v_error = v_diff_jerk_dec + min_acc * t_min + v_diff_jerk_acc
    t_min = (v_error - v_diff_jerk_dec - v_diff_jerk_acc) / min_acc;
  }

  if (t_min > 0) {
    // Necessary to increase deceleration to min_acc
    type = AccelerationType::TRAPEZOID;
  } else {
    const double v_error = target_vel - v0;
    const double v_diff_decel = 0.5 * a0 * calculateTime(a0, 0.0, jerk_acc);
    if (v_diff_decel > v_error || a0 > 0.0) {
      // Necessary to increase deceleration case
      type = AccelerationType::TRIANGLE;
    } else {
      type = AccelerationType::LINEAR;
    }
  }

  double x, v, a, j;
  std::tuple<double, double, double, double> state;

  switch (type) {
    case AccelerationType::TRAPEZOID: {
      // Calculate time
      double t1 = calculateTime(a0, min_acc, jerk_dec);
      double t2 = t_min;
      double t3 = calculateTime(min_acc, 0.0, jerk_acc);
      if (t1 < t_threshold) {
        t1 = 0;
      }
      if (t2 < t_threshold) {
        t2 = 0;
      }
      if (t3 < t_threshold) {
        t3 = 0;
      }

      // Set jerk profile
      jerk_profile.insert(std::make_pair(jerk_dec, t1));
      jerk_profile.insert(std::make_pair(0.0, t2));
      jerk_profile.insert(std::make_pair(jerk_acc, t3));

      // Calculate state = (x, v, a, j) at t = t1 + t2 + t3
      const auto state = updateStateWithJerkConstraint(v0, a0, jerk_profile, t1 + t2 + t3);
      if (!state) {
        return false;
      }
      std::tie(x, v, a, j) = *state;
      break;
    }

    case AccelerationType::TRIANGLE: {
      /*
          v_error = v_diff_from_a0_to_zero + v_diff_from_zero_to_a1 + v_diff_from_a1_to_zero,
          where v_diff_from_zero_to_a1 = 1/2 * ((a1 - 0.0) / jerk_dec) * a1, v_diff_from_a1_to_zero
         = 1/2 * ((0.0 - a1) / jerk_acc) * a1. Thus a1^2 = (v_error - v_diff_from_a0_to_zero) * 2
         * 1.0 / (1.0 / jerk_dec - 1.0 / jerk_acc).
        */
      const double v_error = target_vel - v0;
      const double v_diff_from_a0_to_zero = 0.5 * calculateTime(a0, 0.0, jerk_dec) * a0;
      const double a1_square =
        (v_error - v_diff_from_a0_to_zero) * 2 * 1.0 / (1.0 / jerk_dec - 1.0 / jerk_acc);
      const double a1 = -std::sqrt(a1_square);

      // Calculate time
      double t1 = calculateTime(a0, a1, jerk_dec);
      double t2 = calculateTime(a1, 0.0, jerk_acc);
      if (t1 < t_threshold) {
        t1 = 0;
      }
      if (t2 < t_threshold) {
        t2 = 0;
      }

      // Set jerk profile
      jerk_profile.insert(std::make_pair(jerk_dec, t1));
      jerk_profile.insert(std::make_pair(jerk_acc, t2));

      // Calculate state = (x, v, a, j) at t = t1 + t2
      const auto state = updateStateWithJerkConstraint(v0, a0, jerk_profile, t1 + t2);
      if (!state) {
        return false;
      }
      std::tie(x, v, a, j) = *state;
      break;
    }

    case AccelerationType::LINEAR: {
      // Calculate time
      const double jerk = (0.0 - a0 * a0) / (2 * (target_vel - v0));
      double t1 = calculateTime(a0, 0.0, jerk);
      if (t1 < 0) {
        RCLCPP_WARN(
          rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
          "t1 < 0. unexpected condition.");
        return false;
      } else if (t1 < t_threshold) {
        t1 = 0;
      }

      // Set jerk profile
      jerk_profile.insert(std::make_pair(jerk, t1));

      // Calculate state = (x, v, a, j) at t = t1 + t2
      const auto state = updateStateWithJerkConstraint(v0, a0, jerk_profile, t1);
      if (!state) {
        return false;
      }
      std::tie(x, v, a, j) = *state;
      break;
    }
  }

  constexpr double v_margin = 0.3;  // [m/s]
  constexpr double a_margin = 0.1;  // [m/s^2]
  stop_dist = x;
  if (!isValidStopDist(v, a, target_vel, a_target, v_margin, a_margin)) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "valid error. type = " << static_cast<size_t>(type));
    return false;
  }
  return true;
}

bool isValidStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin)
{
  const double v_min = v_target - std::abs(v_margin);
  const double v_max = v_target + std::abs(v_margin);
  const double a_min = a_target - std::abs(a_margin);
  const double a_max = a_target + std::abs(a_margin);
  if (v_end < v_min || v_max < v_end) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "valid check error! v_target = " << v_target << ", v_end = " << v_end);
    return false;
  }
  if (a_end < a_min || a_max < a_end) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "valid check error! a_target = " << a_target << ", a_end = " << a_end);
    return false;
  }
  return true;
}

std::optional<TrajectoryPoints> applyDecelFilterWithJerkConstraint(
  const TrajectoryPoints & input, const size_t start_index, const double v0, const double a0,
  const double min_acc, const double decel_target_vel,
  const std::map<double, double> & jerk_profile)
{
  double t_total{0.0};
  for (auto & it : jerk_profile) {
    t_total += it.second;
  }

  std::vector<double> ts;
  std::vector<double> xs;
  std::vector<double> vs;
  std::vector<double> as;
  std::vector<double> js;
  const double dt{0.1};
  double x{0.0};
  double v{0.0};
  double a{0.0};
  double j{0.0};

  for (double t = 0.0; t < t_total; t += dt) {
    // Calculate state = (x, v, a, j) at t
    const auto state = updateStateWithJerkConstraint(v0, a0, jerk_profile, t_total);
    if (!state) {
      return {};
    }
    std::tie(x, v, a, j) = *state;
    if (v > 0.0) {
      a = std::max(a, min_acc);
      ts.push_back(t);
      xs.push_back(x);
      vs.push_back(v);
      as.push_back(a);
      js.push_back(j);
    }
  }
  // Calculate state = (x, v, a, j) at t_total
  const auto state = updateStateWithJerkConstraint(v0, a0, jerk_profile, t_total);
  if (!state) {
    return {};
  }
  std::tie(x, v, a, j) = *state;
  if (v > 0.0 && !xs.empty() && xs.back() < x) {
    a = std::max(a, min_acc);
    ts.push_back(t_total);
    xs.push_back(x);
    vs.push_back(v);
    as.push_back(a);
    js.push_back(j);
  }

  const double a_target{0.0};
  const double v_margin{0.3};
  const double a_margin{0.1};
  if (!isValidStopDist(v, a, decel_target_vel, a_target, v_margin, a_margin)) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "validation check error");
    return {};
  }

  TrajectoryPoints output_trajectory{input};

  if (xs.empty()) {
    output_trajectory.at(start_index).longitudinal_velocity_mps = decel_target_vel;
    output_trajectory.at(start_index).acceleration_mps2 = 0.0;
    for (unsigned int i = start_index + 1; i < output_trajectory.size(); ++i) {
      output_trajectory.at(i).longitudinal_velocity_mps = decel_target_vel;
      output_trajectory.at(i).acceleration_mps2 = 0.0;
    }
    return output_trajectory;
  }

  const std::vector<double> distance_all = calcArclengthArray(output_trajectory);
  const auto it_end = std::find_if(
    distance_all.begin(), distance_all.end(), [&xs](double x) { return x > xs.back(); });
  const std::vector<double> distance{distance_all.begin() + start_index, it_end};

  if (
    !interpolation_utils::isIncreasing(xs) || !interpolation_utils::isIncreasing(distance) ||
    !interpolation_utils::isNotDecreasing(xs) || !interpolation_utils::isNotDecreasing(distance)) {
    return {};
  }

  if (
    xs.size() < 2 || vs.size() < 2 || as.size() < 2 || distance.empty() ||
    distance.front() < xs.front() || xs.back() < distance.back()) {
    return {};
  }

  const auto vel_at_wp = interpolation::lerp(xs, vs, distance);
  const auto acc_at_wp = interpolation::lerp(xs, as, distance);

  for (unsigned int i = 0; i < vel_at_wp.size(); ++i) {
    output_trajectory.at(start_index + i).longitudinal_velocity_mps = vel_at_wp.at(i);
    output_trajectory.at(start_index + i).acceleration_mps2 = acc_at_wp.at(i);
  }
  for (unsigned int i = start_index + vel_at_wp.size(); i < output_trajectory.size(); ++i) {
    output_trajectory.at(i).longitudinal_velocity_mps = decel_target_vel;
    output_trajectory.at(i).acceleration_mps2 = 0.0;
  }

  return output_trajectory;
}

std::optional<std::tuple<double, double, double, double>> updateStateWithJerkConstraint(
  const double v0, const double a0, const std::map<double, double> & jerk_profile, const double t)
{
  // constexpr double eps = 1.0E-05;
  double j{0.0};
  double a{a0};
  double v{v0};
  double x{0.0};
  double t_sum{0.0};

  for (auto & it : jerk_profile) {
    j = it.first;
    t_sum += it.second;
    if (t > t_sum) {
      x = integ_x(x, v, a, j, it.second);
      v = integ_v(v, a, j, it.second);
      a = integ_a(a, j, it.second);
    } else {
      const double dt = t - (t_sum - it.second);
      x = integ_x(x, v, a, j, dt);
      v = integ_v(v, a, j, dt);
      a = integ_a(a, j, dt);
      return std::make_tuple(x, v, a, j);
    }
  }

  RCLCPP_WARN_STREAM(
    rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
    "Invalid jerk profile");
  return std::nullopt;
}

std::vector<double> calcVelocityProfileWithConstantJerkAndAccelerationLimit(
  const TrajectoryPoints & trajectory, const double v0, const double a0, const double jerk,
  const double acc_max, const double acc_min)
{
  if (trajectory.empty()) return {};

  std::vector<double> velocities(trajectory.size());
  velocities.at(0) = v0;
  auto curr_v = v0;
  auto curr_a = a0;

  const auto intervals = calcTrajectoryIntervalDistance(trajectory);

  if (intervals.size() + 1 != trajectory.size()) {
    throw std::logic_error("interval calculation result has unexpected array size.");
  }

  for (size_t i = 0; i < intervals.size(); ++i) {
    const auto t = intervals.at(i) / std::max(velocities.at(i), 1.0e-5);
    curr_v = integ_v(curr_v, curr_a, jerk, t);
    velocities.at(i + 1) = curr_v;
    curr_a = std::clamp(integ_a(curr_a, jerk, t), acc_min, acc_max);
  }

  return velocities;
}

double calcStopDistance(const TrajectoryPoints & trajectory, const size_t closest)
{
  const auto idx = motion_utils::searchZeroVelocityIndex(trajectory);

  if (!idx) {
    return std::numeric_limits<double>::max();  // stop point is located far away
  }

  // TODO(Horibe): use arc length distance
  const double stop_dist =
    tier4_autoware_utils::calcDistance2d(trajectory.at(*idx), trajectory.at(closest));

  return stop_dist;
}

}  // namespace trajectory_utils
}  // namespace motion_velocity_smoother
