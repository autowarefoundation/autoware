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

#ifndef SCENE_MODULE__RUN_OUT__PATH_UTILS_HPP_
#define SCENE_MODULE__RUN_OUT__PATH_UTILS_HPP_

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace run_out_utils
{

template <class T>
geometry_msgs::msg::Point findLongitudinalNearestPoint(
  const T & points, const geometry_msgs::msg::Point & src_point,
  const std::vector<geometry_msgs::msg::Point> & target_points)
{
  float min_dist = std::numeric_limits<float>::max();
  geometry_msgs::msg::Point min_dist_point{};

  for (const auto & p : target_points) {
    const float dist = tier4_autoware_utils::calcSignedArcLength(points, src_point, p);
    if (dist < min_dist) {
      min_dist = dist;
      min_dist_point = p;
    }
  }

  return min_dist_point;
}

template <class T>
size_t calcIndexByLength(
  const T & points, const geometry_msgs::msg::Pose & current_pose, const double target_length)
{
  const size_t nearest_index =
    tier4_autoware_utils::findNearestIndex(points, current_pose.position);
  if (target_length < 0) {
    return nearest_index;
  }

  for (size_t i = nearest_index; i < points.size(); i++) {
    double length_sum = tier4_autoware_utils::calcSignedArcLength(points, current_pose.position, i);
    if (length_sum > target_length) {
      return i;
    }
  }

  // reach the end of the points, so return the last index
  return points.size() - 1;
}

template <class T>
size_t calcIndexByLengthReverse(
  const T & points, const geometry_msgs::msg::Point & src_point, const float target_length)
{
  const auto nearest_seg_idx = tier4_autoware_utils::findNearestSegmentIndex(points, src_point);
  if (nearest_seg_idx == 0) {
    return 0;
  }

  for (size_t i = nearest_seg_idx; i > 0; i--) {
    const auto length_sum =
      std::abs(tier4_autoware_utils::calcSignedArcLength(points, src_point, i));
    if (length_sum > target_length) {
      return i + 1;
    }
  }

  // reach the beginning of the path
  return 0;
}

}  // namespace run_out_utils
}  // namespace behavior_velocity_planner
#endif  // SCENE_MODULE__RUN_OUT__PATH_UTILS_HPP_
