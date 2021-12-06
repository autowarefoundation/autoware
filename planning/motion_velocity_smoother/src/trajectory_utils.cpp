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

#include "interpolation/spline_interpolation.hpp"
#include "motion_velocity_smoother/linear_interpolation.hpp"

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
    a[i] = a[i - 1] + autoware_utils::normalizeRadian(da);
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

inline double integ_a(double a0, double j0, double t) { return a0 + j0 * t; }

TrajectoryPoint calcInterpolatedTrajectoryPoint(
  const TrajectoryPoints & trajectory, const Pose & target_pose)
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

  const size_t segment_idx =
    autoware_utils::findNearestSegmentIndex(trajectory, target_pose.position);

  auto v1 = getTransVector3(trajectory.at(segment_idx).pose, trajectory.at(segment_idx + 1).pose);
  auto v2 = getTransVector3(trajectory.at(segment_idx).pose, target_pose);
  // calc internal proportion
  const double prop{std::max(0.0, std::min(1.0, v1.dot(v2) / v1.length2()))};

  auto interpolate = [&prop](double x1, double x2) { return prop * x1 + (1.0 - prop) * x2; };

  {
    const auto & seg_pt = trajectory.at(segment_idx);
    const auto & next_pt = trajectory.at(segment_idx + 1);
    traj_p.longitudinal_velocity_mps =
      interpolate(next_pt.longitudinal_velocity_mps, seg_pt.longitudinal_velocity_mps);
    traj_p.acceleration_mps2 = interpolate(next_pt.acceleration_mps2, seg_pt.acceleration_mps2);
    traj_p.pose.position.x = interpolate(next_pt.pose.position.x, seg_pt.pose.position.x);
    traj_p.pose.position.y = interpolate(next_pt.pose.position.y, seg_pt.pose.position.y);
    traj_p.pose.position.z = interpolate(next_pt.pose.position.z, seg_pt.pose.position.z);
  }

  return traj_p;
}

boost::optional<TrajectoryPoints> extractPathAroundIndex(
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
      dist_sum += autoware_utils::calcDistance2d(trajectory.at(i), trajectory.at(i + 1));
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
      dist_sum += autoware_utils::calcDistance2d(trajectory.at(i), trajectory[i - 1]);
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

  return boost::optional<TrajectoryPoints>(extracted_traj);
}

double calcArcLength(const TrajectoryPoints & path, const int idx1, const int idx2)
{
  if (idx1 == idx2) {  // zero distance
    return 0.0;
  }

  if (
    idx1 < 0 || idx2 < 0 || static_cast<int>(path.size()) - 1 < idx1 ||
    static_cast<int>(path.size()) - 1 < idx2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "invalid index");
    return 0.0;
  }

  const int idx_from = std::min(idx1, idx2);
  const int idx_to = std::max(idx1, idx2);
  double dist_sum = 0.0;
  for (int i = idx_from; i < idx_to; ++i) {
    dist_sum += autoware_utils::calcDistance2d(path.at(i), path.at(i + 1));
  }
  return dist_sum;
}

std::vector<double> calcArclengthArray(const TrajectoryPoints & trajectory)
{
  std::vector<double> arclength;
  double dist = 0.0;
  arclength.clear();
  arclength.push_back(dist);
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    const TrajectoryPoint tp = trajectory.at(i);
    const TrajectoryPoint tp_prev = trajectory.at(i - 1);
    dist += autoware_utils::calcDistance2d(tp.pose, tp_prev.pose);
    arclength.push_back(dist);
  }
  return arclength;
}

std::vector<double> calcTrajectoryIntervalDistance(const TrajectoryPoints & trajectory)
{
  std::vector<double> intervals;
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    const TrajectoryPoint tp = trajectory.at(i);
    const TrajectoryPoint tp_prev = trajectory.at(i - 1);
    const double dist = autoware_utils::calcDistance2d(tp.pose, tp_prev.pose);
    intervals.push_back(dist);
  }
  return intervals;
}

boost::optional<std::vector<double>> calcTrajectoryCurvatureFrom3Points(
  const TrajectoryPoints & trajectory, const size_t & idx_dist)
{
  std::vector<double> k_arr;
  if (trajectory.size() < 2 * idx_dist + 1) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "cannot calc curvature idx_dist = %lu, trajectory.size() = %lu", idx_dist, trajectory.size());
    return {};
  }

  // calculate curvature by circle fitting from three points
  Point p1{};
  Point p2{};
  Point p3{};
  for (size_t i = idx_dist; i < trajectory.size() - idx_dist; ++i) {
    p1.x = trajectory.at(i - idx_dist).pose.position.x;
    p2.x = trajectory.at(i).pose.position.x;
    p3.x = trajectory.at(i + idx_dist).pose.position.x;
    p1.y = trajectory.at(i - idx_dist).pose.position.y;
    p2.y = trajectory.at(i).pose.position.y;
    p3.y = trajectory.at(i + idx_dist).pose.position.y;
    double den = std::max(
      autoware_utils::calcDistance2d(p1, p2) * autoware_utils::calcDistance2d(p2, p3) *
        autoware_utils::calcDistance2d(p3, p1),
      0.0001);
    double curvature = 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    k_arr.push_back(curvature);
  }

  // for debug
  if (k_arr.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "k_arr.size() = 0, something wrong. pls check.");
    return {};
  }

  // first and last curvature is copied from next value
  for (size_t i = 0; i < idx_dist; ++i) {
    k_arr.insert(k_arr.begin(), k_arr.front());
    k_arr.push_back(k_arr.back());
  }
  return boost::optional<std::vector<double>>(k_arr);
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

boost::optional<TrajectoryPoints> applyLinearInterpolation(
  const std::vector<double> & base_index, const TrajectoryPoints & base_trajectory,
  const std::vector<double> & out_index, const bool use_spline_for_pose)
{
  std::vector<double> px, py, pz, pyaw, tlx, taz, alx;
  for (const auto & p : base_trajectory) {
    px.push_back(p.pose.position.x);
    py.push_back(p.pose.position.y);
    pz.push_back(p.pose.position.z);
    pyaw.push_back(tf2::getYaw(p.pose.orientation));
    tlx.push_back(p.longitudinal_velocity_mps);
    taz.push_back(p.heading_rate_rps);
    alx.push_back(p.acceleration_mps2);
  }

  convertEulerAngleToMonotonic(pyaw);

  boost::optional<std::vector<double>> px_p, py_p, pz_p, pyaw_p;
  if (use_spline_for_pose) {
    px_p = interpolation::slerp(base_index, px, out_index);
    py_p = interpolation::slerp(base_index, py, out_index);
    pz_p = interpolation::slerp(base_index, pz, out_index);
    pyaw_p = interpolation::slerp(base_index, pyaw, out_index);
  } else {
    px_p = linear_interpolation::interpolate(base_index, px, out_index);
    py_p = linear_interpolation::interpolate(base_index, py, out_index);
    pz_p = linear_interpolation::interpolate(base_index, pz, out_index);
    pyaw_p = linear_interpolation::interpolate(base_index, pyaw, out_index);
  }
  const auto tlx_p = linear_interpolation::interpolate(base_index, tlx, out_index);
  const auto taz_p = linear_interpolation::interpolate(base_index, taz, out_index);
  const auto alx_p = linear_interpolation::interpolate(base_index, alx, out_index);

  if (!px_p || !py_p || !pz_p || !pyaw_p || !tlx_p || !taz_p || !alx_p) {
    RCLCPP_WARN(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "interpolation error!!");
    return {};
  }

  TrajectoryPoints out_trajectory;
  TrajectoryPoint point;
  for (unsigned int i = 0; i < out_index.size(); ++i) {
    point.pose.position.x = px_p->at(i);
    point.pose.position.y = py_p->at(i);
    point.pose.position.z = pz_p->at(i);
    point.pose.orientation = autoware_utils::createQuaternionFromYaw(pyaw_p->at(i));

    point.longitudinal_velocity_mps = tlx_p->at(i);
    point.heading_rate_rps = taz_p->at(i);
    point.acceleration_mps2 = alx_p->at(i);
    out_trajectory.push_back(point);
  }
  return boost::optional<TrajectoryPoints>(out_trajectory);
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

boost::optional<TrajectoryPoints> applyDecelFilterWithJerkConstraint(
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

  const auto vel_at_wp = linear_interpolation::interpolate(xs, vs, distance);
  const auto acc_at_wp = linear_interpolation::interpolate(xs, as, distance);

  if (!vel_at_wp || !acc_at_wp) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
      "interpolation error");
    return {};
  }

  for (unsigned int i = 0; i < vel_at_wp->size(); ++i) {
    output_trajectory.at(start_index + i).longitudinal_velocity_mps = vel_at_wp->at(i);
    output_trajectory.at(start_index + i).acceleration_mps2 = acc_at_wp->at(i);
  }
  for (unsigned int i = start_index + vel_at_wp->size(); i < output_trajectory.size(); ++i) {
    output_trajectory.at(i).longitudinal_velocity_mps = decel_target_vel;
    output_trajectory.at(i).acceleration_mps2 = 0.0;
  }

  return output_trajectory;
}

boost::optional<std::tuple<double, double, double, double>> updateStateWithJerkConstraint(
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
      const auto state = std::make_tuple(x, v, a, j);
      return boost::optional<std::tuple<double, double, double, double>>(state);
    }
  }

  RCLCPP_WARN_STREAM(
    rclcpp::get_logger("motion_velocity_smoother").get_child("trajectory_utils"),
    "Invalid jerk profile");
  return {};
}

}  // namespace trajectory_utils
}  // namespace motion_velocity_smoother
