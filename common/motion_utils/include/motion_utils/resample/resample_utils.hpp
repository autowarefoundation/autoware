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

#ifndef MOTION_UTILS__RESAMPLE__RESAMPLE_UTILS_HPP_
#define MOTION_UTILS__RESAMPLE__RESAMPLE_UTILS_HPP_

#include <motion_utils/constants.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/geometry/path_with_lane_id_geometry.hpp>

#include <vector>

namespace resample_utils
{
constexpr double CLOSE_S_THRESHOLD = 1e-6;

template <class T>
bool validate_size(const T & points)
{
  if (points.size() < 2) {
    return false;
  }
  return true;
}

template <class T>
bool validate_resampling_range(const T & points, const std::vector<double> & resampling_intervals)
{
  const double points_length = motion_utils::calcArcLength(points);
  if (points_length < resampling_intervals.back()) {
    return false;
  }

  return true;
}

template <class T>
bool validate_points_duplication(const T & points)
{
  for (size_t i = 0; i < points.size() - 1; ++i) {
    const auto & curr_pt = tier4_autoware_utils::getPoint(points.at(i));
    const auto & next_pt = tier4_autoware_utils::getPoint(points.at(i + 1));
    const double ds = tier4_autoware_utils::calcDistance2d(curr_pt, next_pt);
    if (ds < CLOSE_S_THRESHOLD) {
      return false;
    }
  }

  return true;
}

template <class T>
bool validate_arguments(const T & input_points, const std::vector<double> & resampling_intervals)
{
  // Check size of the arguments
  if (!validate_size(input_points)) {
    std::cerr << "The number of input points is less than 2" << std::endl;
    return false;
  } else if (!validate_size(resampling_intervals)) {
    std::cerr << "The number of resampling intervals is less than 2" << std::endl;
    return false;
  }

  // Check resampling range
  if (!validate_resampling_range(input_points, resampling_intervals)) {
    std::cerr << "resampling interval is longer than input points" << std::endl;
    return false;
  }

  // Check duplication
  if (!validate_points_duplication(input_points)) {
    std::cerr << "input points has some duplicated points" << std::endl;
    return false;
  }

  return true;
}

template <class T>
bool validate_arguments(const T & input_points, const double resampling_interval)
{
  // Check size of the arguments
  if (!validate_size(input_points)) {
    std::cerr << "The number of input points is less than 2" << std::endl;
    return false;
  }

  // check resampling interval
  if (resampling_interval < motion_utils::overlap_threshold) {
    std::cerr << "resampling interval is less than " << motion_utils::overlap_threshold
              << std::endl;
    return false;
  }

  // Check duplication
  if (!validate_points_duplication(input_points)) {
    std::cerr << "input points has some duplicated points" << std::endl;
    return false;
  }

  return true;
}
}  // namespace resample_utils

#endif  // MOTION_UTILS__RESAMPLE__RESAMPLE_UTILS_HPP_
