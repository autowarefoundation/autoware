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

#ifndef UTIL_TYPE_HPP_
#define UTIL_TYPE_HPP_

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::util
{

/**
 * @struct
 * @brief  wrapper class of interpolated path with lane id
 */
struct InterpolatedPathInfo
{
  /** the interpolated path */
  autoware_auto_planning_msgs::msg::PathWithLaneId path;
  /** discretization interval of interpolation */
  double ds{0.0};
  /** the intersection lanelet id */
  lanelet::Id lane_id{0};
  /** the associative lane ids of lane_id */
  std::set<lanelet::Id> associative_lane_ids{};
  /** the range of indices for the path points with associative lane id */
  std::optional<std::pair<size_t, size_t>> lane_id_interval{std::nullopt};
};

}  // namespace behavior_velocity_planner::util

#endif  // UTIL_TYPE_HPP_
