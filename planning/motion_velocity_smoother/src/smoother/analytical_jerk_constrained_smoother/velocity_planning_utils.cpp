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

#include "motion_velocity_smoother/smoother/analytical_jerk_constrained_smoother/velocity_planning_utils.hpp"

#include "interpolation/linear_interpolation.hpp"

#include <algorithm>
#include <vector>

namespace motion_velocity_smoother
{
namespace analytical_velocity_planning_utils
{
bool calcStopDistWithJerkAndAccConstraints(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double target_vel, int & type, std::vector<double> & times,
  double & stop_dist)
{
  const double t_min =
    (target_vel - v0 - 0.5 * (0 - a0) / jerk_dec * a0 - 0.5 * min_acc / jerk_dec * min_acc -
     0.5 * (0 - min_acc) / jerk_acc * min_acc) /
    min_acc;

  if (t_min > 0) {
    double t1 = (min_acc - a0) / jerk_dec;
    if (t1 < 0.01) {
      t1 = 0;
    }

    const double a1 = a0 + jerk_dec * t1;
    const double v1 = v0 + a0 * t1 + 0.5 * jerk_dec * t1 * t1;
    const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * jerk_dec * t1 * t1 * t1;

    double t2 = t_min;
    if (t2 < 0.01) {
      t2 = 0;
    }

    const double a2 = a1;
    const double v2 = v1 + a1 * t2;
    const double x2 = x1 + v1 * t2 + 0.5 * a1 * t2 * t2;

    double t3 = (0 - min_acc) / jerk_acc;
    if (t3 < 0.01) {
      t3 = 0;
    }

    const double a3 = a2 + jerk_acc * t3;
    const double v3 = v2 + a2 * t3 + 0.5 * jerk_acc * t3 * t3;
    const double x3 = x2 + v2 * t3 + 0.5 * a2 * t3 * t3 + (1.0 / 6.0) * jerk_acc * t3 * t3 * t3;

    const double a_target = 0.0;
    const double v_margin = 0.3;  // [m/s]
    const double a_margin = 0.1;  // [m/s^2]
    if (!validCheckCalcStopDist(v3, a3, target_vel, a_target, v_margin, a_margin)) {
      RCLCPP_DEBUG(rclcpp::get_logger("velocity_planning_utils"), "Valid check error. type = 1");
      return false;
    }

    type = 1;
    times.push_back(t1);
    times.push_back(t2);
    times.push_back(t3);
    stop_dist = x3;
  } else {
    const double is_decel_needed = 0.5 * (0 - a0) / jerk_acc * a0 - (target_vel - v0);
    if (is_decel_needed > 0 || a0 > 0) {
      const double a1_square = (target_vel - v0 - 0.5 * (0 - a0) / jerk_dec * a0) *
                               (2 * jerk_acc * jerk_dec / (jerk_acc - jerk_dec));
      const double a1 = -std::sqrt(a1_square);

      double t1 = (a1 - a0) / jerk_dec;
      if (t1 < 0.01) {
        t1 = 0;
      }

      const double v1 = v0 + a0 * t1 + 0.5 * jerk_dec * t1 * t1;
      const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * jerk_dec * t1 * t1 * t1;

      double t2 = (0 - a1) / jerk_acc;
      if (t2 < 0.01) {
        t2 = 0;
      }

      const double a2 = a1 + jerk_acc * t2;
      const double v2 = v1 + a1 * t2 + 0.5 * jerk_acc * t2 * t2;
      const double x2 = x1 + v1 * t2 + 0.5 * a1 * t2 * t2 + (1.0 / 6.0) * jerk_acc * t2 * t2 * t2;

      const double a_target = 0.0;
      const double v_margin = 0.3;
      const double a_margin = 0.1;
      if (!validCheckCalcStopDist(v2, a2, target_vel, a_target, v_margin, a_margin)) {
        RCLCPP_DEBUG(rclcpp::get_logger("velocity_planning_utils"), "Valid check error. type = 2");
        return false;
      }

      type = 2;
      times.push_back(t1);
      times.push_back(t2);
      stop_dist = x2;
    } else {
      double t1 = (0 - a0) / jerk_acc;
      if (t1 < 0) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("velocity_planning_utils"), "t1 < 0. unexpected condition.");
        return false;
      }
      if (t1 < 0.01) {
        t1 = 0;
      }

      const double a1 = a0 + jerk_acc * t1;
      const double v1 = v0 + a0 * t1 + 0.5 * jerk_acc * t1 * t1;
      const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * jerk_acc * t1 * t1 * t1;

      const double a_target = 0.0;
      const double v_margin = 0.3;
      const double a_margin = 0.1;
      if (!validCheckCalcStopDist(v1, a1, target_vel, a_target, v_margin, a_margin)) {
        RCLCPP_DEBUG(rclcpp::get_logger("velocity_planning_utils"), "Valid check error. type = 3");
        return false;
      }

      type = 3;
      times.push_back(t1);
      stop_dist = x1;
    }
  }
  return true;
}

bool validCheckCalcStopDist(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin)
{
  const double v_min = v_target - std::abs(v_margin);
  const double v_max = v_target + std::abs(v_margin);
  const double a_min = a_target - std::abs(a_margin);
  const double a_max = a_target + std::abs(a_margin);
  if (v_end < v_min || v_max < v_end) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("velocity_planning_utils"), "Valid check error! v_target = %f, v_end = %f",
      v_target, v_end);
    return false;
  }
  if (a_end < a_min || a_max < a_end) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("velocity_planning_utils"), "Valid check error! a_target = %f, a_end = %f",
      a_target, a_end);
    return false;
  }
  return true;
}

bool calcStopVelocityWithConstantJerkAccLimit(
  const double v0, const double a0, const double jerk_acc, const double jerk_dec,
  const double min_acc, const double decel_target_vel, const int type,
  const std::vector<double> & times, const size_t start_index, TrajectoryPoints & output_trajectory)
{
  const double t_total = std::accumulate(times.begin(), times.end(), 0.0);
  std::vector<double> ts, xs, vs, as, js;
  const double dt = 0.1;
  double x = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;

  for (double t = 0.0; t < t_total; t += dt) {
    updateStopVelocityStatus(v0, a0, jerk_acc, jerk_dec, type, times, t, x, v, a, j);
    if (v > 0.0) {
      a = std::max(a, min_acc);
      ts.push_back(t);
      xs.push_back(x);
      vs.push_back(v);
      as.push_back(a);
      js.push_back(j);
    }
  }
  updateStopVelocityStatus(v0, a0, jerk_acc, jerk_dec, type, times, t_total, x, v, a, j);
  if (v > 0.0 && !xs.empty() && xs.back() < x) {
    a = std::max(a, min_acc);
    ts.push_back(t_total);
    xs.push_back(x);
    vs.push_back(v);
    as.push_back(a);
    js.push_back(j);
  }

  // for debug
  std::stringstream ss;
  for (unsigned int i = 0; i < ts.size(); ++i) {
    ss << "t: " << ts.at(i) << ", x: " << xs.at(i) << ", v: " << vs.at(i) << ", a: " << as.at(i)
       << ", j: " << js.at(i) << std::endl;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("velocity_planning_utils"), "Calculate stop velocity. %s", ss.str().c_str());

  const double a_target = 0.0;
  const double v_margin = 0.3;
  const double a_margin = 0.1;
  if (!validCheckCalcStopDist(v, a, decel_target_vel, a_target, v_margin, a_margin)) {
    return false;
  }

  if (xs.empty()) {
    for (size_t i = start_index; i < output_trajectory.size(); ++i) {
      output_trajectory.at(i).longitudinal_velocity_mps = decel_target_vel;
      output_trajectory.at(i).acceleration_mps2 = 0.0;
    }
    return true;
  }

  double distance = 0.0;
  std::vector<double> distances;
  distances.push_back(distance);
  for (size_t i = start_index; i < output_trajectory.size() - 1; ++i) {
    distance +=
      tier4_autoware_utils::calcDistance2d(output_trajectory.at(i), output_trajectory.at(i + 1));
    if (distance > xs.back()) {
      break;
    }
    distances.push_back(distance);
  }

  if (
    !interpolation_utils::isIncreasing(xs) || !interpolation_utils::isIncreasing(distances) ||
    !interpolation_utils::isNotDecreasing(xs) || !interpolation_utils::isNotDecreasing(distances)) {
    return false;
  }

  if (
    xs.size() < 2 || vs.size() < 2 || as.size() < 2 || js.size() < 2 || distances.empty() ||
    distances.front() < xs.front() || xs.back() < distances.back()) {
    return false;
  }

  const auto vel_at_wp = interpolation::lerp(xs, vs, distances);
  const auto acc_at_wp = interpolation::lerp(xs, as, distances);
  const auto jerk_at_wp = interpolation::lerp(xs, js, distances);

  // for debug
  std::stringstream ssi;
  for (unsigned int i = 0; i < distances.size(); ++i) {
    ssi << "d: " << distances.at(i) << ", v: " << vel_at_wp.at(i) << ", a: " << acc_at_wp.at(i)
        << ", j: " << jerk_at_wp.at(i) << std::endl;
  }
  RCLCPP_DEBUG(
    rclcpp::get_logger("velocity_planning_utils"), "Interpolated = %s", ssi.str().c_str());

  for (size_t i = 0; i < vel_at_wp.size(); ++i) {
    output_trajectory.at(start_index + i).longitudinal_velocity_mps = vel_at_wp.at(i);
    output_trajectory.at(start_index + i).acceleration_mps2 = acc_at_wp.at(i);
  }
  for (size_t i = start_index + vel_at_wp.size(); i < output_trajectory.size(); ++i) {
    output_trajectory.at(i).longitudinal_velocity_mps = decel_target_vel;
    output_trajectory.at(i).acceleration_mps2 = 0.0;
  }

  return true;
}

void updateStopVelocityStatus(
  double v0, double a0, double jerk_acc, double jerk_dec, int type, std::vector<double> times,
  double t, double & x, double & v, double & a, double & j)
{
  if (type == 1) {
    if (0 <= t && t < times.at(0)) {
      j = jerk_dec;
      a = integ_a(a0, j, t);
      v = integ_v(v0, a0, j, t);
      x = integ_x(0, v0, a0, j, t);
    } else if (times.at(0) <= t && t < times.at(0) + times.at(1)) {
      const double t1 = times.at(0);
      const double a1 = integ_a(a0, jerk_dec, t1);
      const double v1 = integ_v(v0, a0, jerk_dec, t1);
      const double x1 = integ_x(0, v0, a0, jerk_dec, t1);

      const double dt = t - t1;
      j = 0;
      a = integ_a(a1, j, dt);
      v = integ_v(v1, a1, j, dt);
      x = integ_x(x1, v1, a1, j, dt);
    } else if (times.at(0) + times.at(1) <= t && t <= times.at(0) + times.at(1) + times.at(2)) {
      const double t1 = times.at(0);
      const double a1 = integ_a(a0, jerk_dec, t1);
      const double v1 = integ_v(v0, a0, jerk_dec, t1);
      const double x1 = integ_x(0, v0, a0, jerk_dec, t1);

      const double t2 = times.at(1);
      const double a2 = integ_a(a1, 0, t2);
      const double v2 = integ_v(v1, a1, 0, t2);
      const double x2 = integ_x(x1, v1, a1, 0, t2);

      const double dt = t - (t1 + t2);
      j = jerk_acc;
      a = integ_a(a2, j, dt);
      v = integ_v(v2, a2, j, dt);
      x = integ_x(x2, v2, a2, j, dt);
    }
  } else if (type == 2) {
    if (0 <= t && t < times.at(0)) {
      j = jerk_dec;
      a = integ_a(a0, j, t);
      v = integ_v(v0, a0, j, t);
      x = integ_x(0, v0, a0, j, t);
    } else if (times.at(0) <= t && t <= times.at(0) + times.at(1)) {
      const double t1 = times.at(0);
      const double a1 = integ_a(a0, jerk_dec, t1);
      const double v1 = integ_v(v0, a0, jerk_dec, t1);
      const double x1 = integ_x(0, v0, a0, jerk_dec, t1);

      const double dt = t - t1;
      j = jerk_acc;
      a = integ_a(a1, j, dt);
      v = integ_v(v1, a1, j, dt);
      x = integ_x(x1, v1, a1, j, dt);
    }
  } else if (type == 3) {
    if (0 <= t && t <= times.at(0)) {
      j = jerk_acc;
      a = integ_a(a0, j, t);
      v = integ_v(v0, a0, j, t);
      x = integ_x(0, v0, a0, j, t);
    }
  } else {
  }
}

double integ_x(double x0, double v0, double a0, double j0, double t)
{
  return x0 + v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * j0 * t * t * t;
}

double integ_v(double v0, double a0, double j0, double t)
{
  return v0 + a0 * t + 0.5 * j0 * t * t;
}

double integ_a(double a0, double j0, double t)
{
  return a0 + j0 * t;
}

}  // namespace analytical_velocity_planning_utils
}  // namespace motion_velocity_smoother
