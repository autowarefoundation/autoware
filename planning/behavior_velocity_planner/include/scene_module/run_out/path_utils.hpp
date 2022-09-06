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
    const float dist = motion_utils::calcSignedArcLength(points, src_point, p);
    if (dist < min_dist) {
      min_dist = dist;
      min_dist_point = p;
    }
  }

  return min_dist_point;
}

}  // namespace run_out_utils
}  // namespace behavior_velocity_planner
#endif  // SCENE_MODULE__RUN_OUT__PATH_UTILS_HPP_
