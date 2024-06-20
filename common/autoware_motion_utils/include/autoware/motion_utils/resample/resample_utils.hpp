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

#ifndef AUTOWARE__MOTION_UTILS__RESAMPLE__RESAMPLE_UTILS_HPP_
#define AUTOWARE__MOTION_UTILS__RESAMPLE__RESAMPLE_UTILS_HPP_

#include "autoware/universe_utils/system/backtrace.hpp"

#include <autoware/motion_utils/constants.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <vector>

namespace resample_utils
{
constexpr double close_s_threshold = 1e-6;

static inline rclcpp::Logger get_logger()
{
  constexpr const char * logger{"autoware_motion_utils.resample_utils"};
  return rclcpp::get_logger(logger);
}

template <class T>
bool validate_size(const T & points)
{
  return points.size() >= 2;
}

template <class T>
bool validate_resampling_range(const T & points, const std::vector<double> & resampling_intervals)
{
  const double points_length = autoware::motion_utils::calcArcLength(points);
  return points_length >= resampling_intervals.back();
}

template <class T>
bool validate_points_duplication(const T & points)
{
  for (size_t i = 0; i < points.size() - 1; ++i) {
    const auto & curr_pt = autoware::universe_utils::getPoint(points.at(i));
    const auto & next_pt = autoware::universe_utils::getPoint(points.at(i + 1));
    const double ds = autoware::universe_utils::calcDistance2d(curr_pt, next_pt);
    if (ds < close_s_threshold) {
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
    RCLCPP_DEBUG(get_logger(), "invalid argument: The number of input points is less than 2");
    autoware::universe_utils::print_backtrace();
    return false;
  }
  if (!validate_size(resampling_intervals)) {
    RCLCPP_DEBUG(
      get_logger(), "invalid argument: The number of resampling intervals is less than 2");
    autoware::universe_utils::print_backtrace();
    return false;
  }

  // Check resampling range
  if (!validate_resampling_range(input_points, resampling_intervals)) {
    RCLCPP_DEBUG(get_logger(), "invalid argument: resampling interval is longer than input points");
    autoware::universe_utils::print_backtrace();
    return false;
  }

  // Check duplication
  if (!validate_points_duplication(input_points)) {
    RCLCPP_DEBUG(get_logger(), "invalid argument: input points has some duplicated points");
    autoware::universe_utils::print_backtrace();
    return false;
  }

  return true;
}

template <class T>
bool validate_arguments(const T & input_points, const double resampling_interval)
{
  // Check size of the arguments
  if (!validate_size(input_points)) {
    RCLCPP_DEBUG(get_logger(), "invalid argument: The number of input points is less than 2");
    autoware::universe_utils::print_backtrace();
    return false;
  }

  // check resampling interval
  if (resampling_interval < autoware::motion_utils::overlap_threshold) {
    RCLCPP_DEBUG(
      get_logger(), "invalid argument: resampling interval is less than %f",
      autoware::motion_utils::overlap_threshold);
    autoware::universe_utils::print_backtrace();
    return false;
  }

  // Check duplication
  if (!validate_points_duplication(input_points)) {
    RCLCPP_DEBUG(get_logger(), "invalid argument: input points has some duplicated points");
    autoware::universe_utils::print_backtrace();
    return false;
  }

  return true;
}
}  // namespace resample_utils

#endif  // AUTOWARE__MOTION_UTILS__RESAMPLE__RESAMPLE_UTILS_HPP_
