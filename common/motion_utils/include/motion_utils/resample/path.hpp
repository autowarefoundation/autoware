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

#ifndef MOTION_UTILS__RESAMPLE__PATH_HPP_
#define MOTION_UTILS__RESAMPLE__PATH_HPP_

#include "interpolation/interpolation_utils.hpp"
#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/zero_order_hold.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/pose_deviation.hpp"
#include "tier4_autoware_utils/math/constants.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

namespace motion_utils
{
/**
 * @brief A resampling function for a path. Note that in a default setting, position xy are
 * resampled by spline interpolation, position z are resampled by linear interpolation, longitudinal
 * and lateral velocity are resampled by zero_order_hold, and heading rate is resampled by linear
 * interpolation. Orientation of the resampled path are calculated by a forward difference method
 * based on the interpolated position x and y.
 * @param input_path input path to resample
 * @param resampled_arclength arclength that contains length of each resampling points from initial
 * point
 * @param use_lerp_for_xy If this is true, it uses linear interpolation to resample position x and
 * y. Otherwise, it uses spline interpolation
 * @param use_lerp_for_z If this is true, it uses linear interpolation to resample position z.
 * Otherwise, it uses spline interpolation
 * @param use_zero_order_hold_for_v If this is true, it uses zero_order_hold to resample
 * longitudinal and lateral velocity. Otherwise, it uses linear interpolation
 * @return resampled path
 */
inline autoware_auto_planning_msgs::msg::Path resamplePath(
  const autoware_auto_planning_msgs::msg::Path & input_path,
  const std::vector<double> & resampled_arclength, const bool use_lerp_for_xy = false,
  const bool use_lerp_for_z = true, const bool use_zero_order_hold_for_v = true)
{
  // Check vector size and if out_arclength have the end point of the path
  const double input_path_len = motion_utils::calcArcLength(input_path.points);
  if (
    input_path.points.size() < 2 || resampled_arclength.size() < 2 ||
    input_path_len < resampled_arclength.back()) {
    std::cerr
      << "[motion_utils]: input path size, input path length or resampled arclength is wrong"
      << std::endl;
    return input_path;
  }

  // Input Path Information
  std::vector<double> input_arclength(input_path.points.size());
  std::vector<double> x(input_path.points.size());
  std::vector<double> y(input_path.points.size());
  std::vector<double> z(input_path.points.size());
  std::vector<double> v_lon(input_path.points.size());
  std::vector<double> v_lat(input_path.points.size());
  std::vector<double> heading_rate(input_path.points.size());

  input_arclength.front() = 0.0;
  x.front() = input_path.points.front().pose.position.x;
  y.front() = input_path.points.front().pose.position.y;
  z.front() = input_path.points.front().pose.position.z;
  v_lon.front() = input_path.points.front().longitudinal_velocity_mps;
  v_lat.front() = input_path.points.front().lateral_velocity_mps;
  heading_rate.front() = input_path.points.front().heading_rate_rps;
  for (size_t i = 1; i < input_path.points.size(); ++i) {
    const auto & prev_pt = input_path.points.at(i - 1);
    const auto & curr_pt = input_path.points.at(i);
    const double ds =
      tier4_autoware_utils::calcDistance2d(prev_pt.pose.position, curr_pt.pose.position);
    input_arclength.at(i) = ds + input_arclength.at(i - 1);
    x.at(i) = curr_pt.pose.position.x;
    y.at(i) = curr_pt.pose.position.y;
    z.at(i) = curr_pt.pose.position.z;
    v_lon.at(i) = curr_pt.longitudinal_velocity_mps;
    v_lat.at(i) = curr_pt.lateral_velocity_mps;
    heading_rate.at(i) = curr_pt.heading_rate_rps;
  }

  // Interpolate
  const auto lerp = [&](const auto & input) {
    return interpolation::lerp(input_arclength, input, resampled_arclength);
  };
  const auto slerp = [&](const auto & input) {
    return interpolation::slerp(input_arclength, input, resampled_arclength);
  };
  const auto zoh = [&](const auto & input) {
    return interpolation::zero_order_hold(input_arclength, input, resampled_arclength);
  };

  const auto interpolated_x = use_lerp_for_xy ? lerp(x) : slerp(x);
  const auto interpolated_y = use_lerp_for_xy ? lerp(y) : slerp(y);
  const auto interpolated_z = use_lerp_for_z ? lerp(z) : slerp(z);
  const auto interpolated_v_lon = use_zero_order_hold_for_v ? zoh(v_lon) : lerp(v_lon);
  const auto interpolated_v_lat = use_zero_order_hold_for_v ? zoh(v_lat) : lerp(v_lat);
  const auto interpolated_heading_rate = lerp(heading_rate);

  autoware_auto_planning_msgs::msg::Path interpolated_path;
  interpolated_path.header = input_path.header;
  interpolated_path.drivable_area = input_path.drivable_area;
  interpolated_path.points.resize(interpolated_x.size());

  // Insert Position, Velocity and Heading Rate
  for (size_t i = 0; i < interpolated_path.points.size(); ++i) {
    autoware_auto_planning_msgs::msg::PathPoint path_point;
    path_point.pose.position.x = interpolated_x.at(i);
    path_point.pose.position.y = interpolated_y.at(i);
    path_point.pose.position.z = interpolated_z.at(i);
    path_point.longitudinal_velocity_mps = interpolated_v_lon.at(i);
    path_point.lateral_velocity_mps = interpolated_v_lat.at(i);
    path_point.heading_rate_rps = interpolated_heading_rate.at(i);
    interpolated_path.points.at(i) = path_point;
  }

  // Insert Orientation
  for (size_t i = 0; i < interpolated_path.points.size() - 1; ++i) {
    const auto & src_point = interpolated_path.points.at(i).pose.position;
    const auto & dst_point = interpolated_path.points.at(i + 1).pose.position;
    const double pitch = tier4_autoware_utils::calcElevationAngle(src_point, dst_point);
    const double yaw = tier4_autoware_utils::calcAzimuthAngle(src_point, dst_point);
    interpolated_path.points.at(i).pose.orientation =
      tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, yaw);
    if (i == interpolated_path.points.size() - 2) {
      // Terminal Orientation is same as the point before it
      interpolated_path.points.at(i + 1).pose.orientation =
        interpolated_path.points.at(i).pose.orientation;
    }
  }

  return interpolated_path;
}
}  // namespace motion_utils

#endif  // MOTION_UTILS__RESAMPLE__PATH_HPP_
