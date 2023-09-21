// Copyright 2022 Tier IV, Inc.
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

#include "planning_validator/utils.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace planning_validator
{
using tier4_autoware_utils::calcCurvature;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getPoint;

namespace
{
void takeBigger(double & v_max, size_t & i_max, double v, size_t i)
{
  if (v_max < v) {
    v_max = v;
    i_max = i;
  }
}
void takeSmaller(double & v_min, size_t & i_min, double v, size_t i)
{
  if (v_min > v) {
    v_min = v;
    i_min = i;
  }
}
}  // namespace

std::pair<double, size_t> getMaxValAndIdx(const std::vector<double> & v)
{
  const auto iter = std::max_element(v.begin(), v.end());
  const auto idx = std::distance(v.begin(), iter);
  return {*iter, idx};
}

std::pair<double, size_t> getMinValAndIdx(const std::vector<double> & v)
{
  const auto iter = std::min_element(v.begin(), v.end());
  const auto idx = std::distance(v.begin(), iter);
  return {*iter, idx};
}

std::pair<double, size_t> getAbsMaxValAndIdx(const std::vector<double> & v)
{
  const auto iter = std::max_element(
    v.begin(), v.end(), [](const auto & a, const auto & b) { return std::abs(a) < std::abs(b); });
  const auto idx = std::distance(v.begin(), iter);
  return {std::abs(*iter), idx};
}

// Do not interpolate.
Trajectory resampleTrajectory(const Trajectory & trajectory, const double min_interval)
{
  Trajectory resampled;
  resampled.header = trajectory.header;

  if (trajectory.points.empty()) {
    return resampled;
  }

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto prev = resampled.points.back();
    const auto curr = trajectory.points.at(i);
    if (calcDistance2d(prev, curr) > min_interval) {
      resampled.points.push_back(curr);
    }
  }
  return resampled;
}

// calculate curvature from three points with curvature_distance
void calcCurvature(
  const Trajectory & trajectory, std::vector<double> & curvature_arr,
  const double curvature_distance)
{
  if (trajectory.points.size() < 3) {
    curvature_arr = std::vector<double>(trajectory.points.size(), 0.0);
    return;
  }

  // calc arc length array: arc_length(3) - arc_length(0) is distance from point(3) to point(0)
  std::vector<double> arc_length(trajectory.points.size(), 0.0);
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    arc_length.at(i) =
      arc_length.at(i - 1) + calcDistance2d(trajectory.points.at(i - 1), trajectory.points.at(i));
  }

  // initialize with 0 curvature
  curvature_arr = std::vector<double>(trajectory.points.size(), 0.0);

  size_t first_distant_index = 0;
  size_t last_distant_index = trajectory.points.size() - 1;
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    // find the previous point
    size_t prev_idx = 0;
    for (size_t j = i - 1; j > 0; --j) {
      if (arc_length.at(i) - arc_length.at(j) > curvature_distance) {
        if (first_distant_index == 0) {
          first_distant_index = i;  // save first index that meets distance requirement
        }
        prev_idx = j;
        break;
      }
    }

    // find the next point
    size_t next_idx = trajectory.points.size() - 1;
    for (size_t j = i + 1; j < trajectory.points.size(); ++j) {
      if (arc_length.at(j) - arc_length.at(i) > curvature_distance) {
        last_distant_index = i;  // save last index that meets distance requirement
        next_idx = j;
        break;
      }
    }

    const auto p1 = getPoint(trajectory.points.at(prev_idx));
    const auto p2 = getPoint(trajectory.points.at(i));
    const auto p3 = getPoint(trajectory.points.at(next_idx));
    try {
      curvature_arr.at(i) = tier4_autoware_utils::calcCurvature(p1, p2, p3);
    } catch (...) {
      curvature_arr.at(i) = 0.0;  // maybe distance is too close
    }
  }

  // use previous or last curvature where the distance is not enough
  for (size_t i = first_distant_index; i > 0; --i) {
    curvature_arr.at(i - 1) = curvature_arr.at(i);
  }
  for (size_t i = last_distant_index; i < curvature_arr.size() - 1; ++i) {
    curvature_arr.at(i + 1) = curvature_arr.at(i);
  }
}

std::pair<double, size_t> calcMaxCurvature(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 3) {
    return {0.0, 0};
  }

  std::vector<double> curvature_arr;
  calcCurvature(trajectory, curvature_arr);

  const auto max_curvature_it = std::max_element(curvature_arr.begin(), curvature_arr.end());
  const size_t index = std::distance(curvature_arr.begin(), max_curvature_it);

  return {*max_curvature_it, index};
}

std::pair<double, size_t> calcMaxIntervalDistance(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 2) {
    return {0.0, 0};
  }

  double max_interval_distances = 0.0;
  size_t max_index = 0;
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto d = calcDistance2d(trajectory.points.at(i), trajectory.points.at(i - 1));
    if (max_interval_distances < std::abs(d)) {
      takeBigger(max_interval_distances, max_index, std::abs(d), i);
    }
  }
  return {max_interval_distances, max_index};
}

std::pair<double, size_t> calcMaxLateralAcceleration(const Trajectory & trajectory)
{
  std::vector<double> curvatures;
  calcCurvature(trajectory, curvatures);

  double max_lat_acc = 0.0;
  size_t max_index = 0;
  for (size_t i = 0; i < curvatures.size(); ++i) {
    const auto v = trajectory.points.at(i).longitudinal_velocity_mps;
    const auto lat_acc = v * v * curvatures.at(i);
    takeBigger(max_lat_acc, max_index, std::abs(lat_acc), i);
  }
  return {max_lat_acc, max_index};
}

std::pair<double, size_t> getMaxLongitudinalAcc(const Trajectory & trajectory)
{
  double max_acc = 0.0;
  size_t max_index = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    takeBigger(max_acc, max_index, trajectory.points.at(i).acceleration_mps2, i);
  }
  return {max_acc, max_index};
}

std::pair<double, size_t> getMinLongitudinalAcc(const Trajectory & trajectory)
{
  double min_acc = 0.0;
  size_t min_index = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    takeSmaller(min_acc, min_index, trajectory.points.at(i).acceleration_mps2, i);
  }
  return {min_acc, min_index};
}

std::pair<double, size_t> calcMaxRelativeAngles(const Trajectory & trajectory)
{
  // We need at least three points to compute relative angle
  const size_t relative_angle_points_num = 3;
  if (trajectory.points.size() < relative_angle_points_num) {
    return {0.0, 0};
  }

  double max_relative_angles = 0.0;
  size_t max_index = 0;

  for (size_t i = 0; i <= trajectory.points.size() - relative_angle_points_num; ++i) {
    const auto & p1 = trajectory.points.at(i).pose.position;
    const auto & p2 = trajectory.points.at(i + 1).pose.position;
    const auto & p3 = trajectory.points.at(i + 2).pose.position;

    const auto angle_a = tier4_autoware_utils::calcAzimuthAngle(p1, p2);
    const auto angle_b = tier4_autoware_utils::calcAzimuthAngle(p2, p3);

    // convert relative angle to [-pi ~ pi]
    const auto relative_angle = std::abs(tier4_autoware_utils::normalizeRadian(angle_b - angle_a));

    takeBigger(max_relative_angles, max_index, std::abs(relative_angle), i);
  }

  return {max_relative_angles, max_index};
}

void calcSteeringAngles(
  const Trajectory & trajectory, const double wheelbase, std::vector<double> & steering_array)
{
  const auto curvatureToSteering = [](const auto k, const auto wheelbase) {
    return std::atan(k * wheelbase);
  };

  std::vector<double> curvatures;
  calcCurvature(trajectory, curvatures);

  steering_array.clear();
  for (const auto k : curvatures) {
    steering_array.push_back(curvatureToSteering(k, wheelbase));
  }
}

std::pair<double, size_t> calcMaxSteeringAngles(
  const Trajectory & trajectory, const double wheelbase)
{
  std::vector<double> steering_array;
  calcSteeringAngles(trajectory, wheelbase, steering_array);

  return getAbsMaxValAndIdx(steering_array);
}

std::pair<double, size_t> calcMaxSteeringRates(
  const Trajectory & trajectory, const double wheelbase)
{
  if (trajectory.points.size() < 1) {
    return {0.0, 0};
  }

  std::vector<double> steering_array;
  calcSteeringAngles(trajectory, wheelbase, steering_array);

  double max_steering_rate = 0.0;
  size_t max_index = 0;
  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto & p_prev = trajectory.points.at(i);
    const auto & p_next = trajectory.points.at(i + 1);
    const auto delta_s = calcDistance2d(p_prev, p_next);
    const auto v = 0.5 * (p_next.longitudinal_velocity_mps + p_prev.longitudinal_velocity_mps);
    const auto dt = delta_s / std::max(v, 1.0e-5);

    const auto steer_prev = steering_array.at(i);
    const auto steer_next = steering_array.at(i + 1);

    const auto steer_rate = (steer_next - steer_prev) / dt;
    takeBigger(max_steering_rate, max_index, steer_rate, i);
  }

  return {max_steering_rate, max_index};
}

bool checkFinite(const TrajectoryPoint & point)
{
  const auto & p = point.pose.position;
  const auto & o = point.pose.orientation;

  using std::isfinite;
  const bool p_result = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
  const bool quat_result = isfinite(o.x) && isfinite(o.y) && isfinite(o.z) && isfinite(o.w);
  const bool v_result = isfinite(point.longitudinal_velocity_mps);
  const bool w_result = isfinite(point.heading_rate_rps);
  const bool a_result = isfinite(point.acceleration_mps2);

  return quat_result && p_result && v_result && w_result && a_result;
}

void shiftPose(geometry_msgs::msg::Pose & pose, double longitudinal)
{
  const auto yaw = tf2::getYaw(pose.orientation);
  pose.position.x += std::cos(yaw) * longitudinal;
  pose.position.y += std::sin(yaw) * longitudinal;
}

}  // namespace planning_validator
