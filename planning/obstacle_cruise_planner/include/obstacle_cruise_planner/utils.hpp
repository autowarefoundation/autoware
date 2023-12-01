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

#ifndef OBSTACLE_CRUISE_PLANNER__UTILS_HPP_
#define OBSTACLE_CRUISE_PLANNER__UTILS_HPP_

#include "common_structs.hpp"
#include "obstacle_cruise_planner/type_alias.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace obstacle_cruise_utils
{
Marker getObjectMarker(
  const geometry_msgs::msg::Pose & obj_pose, size_t idx, const std::string & ns, const double r,
  const double g, const double b);

PoseWithStamp getCurrentObjectPose(
  const PredictedObject & predicted_object, const rclcpp::Time & obj_base_time,
  const rclcpp::Time & current_time, const bool use_prediction);

std::optional<StopObstacle> getClosestStopObstacle(
  const std::vector<StopObstacle> & stop_obstacles);

template <class T>
size_t getIndexWithLongitudinalOffset(
  const T & points, const double longitudinal_offset, std::optional<size_t> start_idx)
{
  if (points.empty()) {
    throw std::logic_error("points is empty.");
  }

  if (start_idx) {
    if (/*start_idx.get() < 0 || */ points.size() <= *start_idx) {
      throw std::out_of_range("start_idx is out of range.");
    }
  } else {
    if (longitudinal_offset > 0) {
      start_idx = 0;
    } else {
      start_idx = points.size() - 1;
    }
  }

  double sum_length = 0.0;
  if (longitudinal_offset > 0) {
    for (size_t i = *start_idx; i < points.size() - 1; ++i) {
      const double segment_length =
        tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
      sum_length += segment_length;
      if (sum_length >= longitudinal_offset) {
        const double back_length = sum_length - longitudinal_offset;
        const double front_length = segment_length - back_length;
        if (front_length < back_length) {
          return i;
        } else {
          return i + 1;
        }
      }
    }
    return points.size() - 1;
  }

  for (size_t i = *start_idx; 0 < i; --i) {
    const double segment_length =
      tier4_autoware_utils::calcDistance2d(points.at(i - 1), points.at(i));
    sum_length += segment_length;
    if (sum_length >= -longitudinal_offset) {
      const double back_length = sum_length + longitudinal_offset;
      const double front_length = segment_length - back_length;
      if (front_length < back_length) {
        return i;
      } else {
        return i - 1;
      }
    }
  }
  return 0;
}
}  // namespace obstacle_cruise_utils

#endif  // OBSTACLE_CRUISE_PLANNER__UTILS_HPP_
