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

#ifndef INTERPOLATED_PATH_INFO_HPP_
#define INTERPOLATED_PATH_INFO_HPP_

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <optional>
#include <set>
#include <utility>

namespace autoware::behavior_velocity_planner
{

/**
 * @brief  wrapper class of interpolated path with lane id
 */
struct InterpolatedPathInfo
{
  /** the interpolated path */
  tier4_planning_msgs::msg::PathWithLaneId path;
  /** discretization interval of interpolation */
  double ds{0.0};
  /** the intersection lanelet id */
  lanelet::Id lane_id{0};
  /** the associative lane ids of lane_id */
  std::set<lanelet::Id> associative_lane_ids{};
  /** the range of indices for the path points with associative lane id */
  std::optional<std::pair<size_t, size_t>> lane_id_interval{std::nullopt};
};

}  // namespace autoware::behavior_velocity_planner

#endif  // INTERPOLATED_PATH_INFO_HPP_
