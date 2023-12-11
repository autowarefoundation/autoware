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

#ifndef MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
#define MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_

#include "autoware_auto_planning_msgs/msg/detail/path_with_lane_id__struct.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <utility>
namespace motion_utils
{
std::optional<std::pair<size_t, size_t>> getPathIndexRangeWithLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int64_t target_lane_id);

size_t findNearestIndexFromLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & pos, const int64_t lane_id);

size_t findNearestSegmentIndexFromLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & pos, const int64_t lane_id);

// @brief Calculates the path to be followed by the rear wheel center in order to make the vehicle
// center follow the input path
// @param [in] path with position to be followed by the center of the vehicle
// @param [out] PathWithLaneId to be followed by the rear wheel center follow to make the vehicle
// center follow the input path NOTE: rear_to_cog is supposed to be positive
autoware_auto_planning_msgs::msg::PathWithLaneId convertToRearWheelCenter(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const double rear_to_cog,
  const bool enable_last_point_compensation = true);
}  // namespace motion_utils

#endif  // MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
