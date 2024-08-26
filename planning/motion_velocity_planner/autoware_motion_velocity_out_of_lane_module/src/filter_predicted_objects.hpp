// Copyright 2024-2024 TIER IV, Inc.
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

#ifndef FILTER_PREDICTED_OBJECTS_HPP_
#define FILTER_PREDICTED_OBJECTS_HPP_

#include "types.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>

#include <optional>

namespace autoware::motion_velocity_planner::out_of_lane
{
/// @brief cut a predicted path beyond the given stop line
/// @param [inout] predicted_path predicted path to cut
/// @param [in] stop_line stop line used for cutting
/// @param [in] object_front_overhang extra distance to cut ahead of the stop line
void cut_predicted_path_beyond_line(
  autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const universe_utils::LineString2d & stop_line, const double object_front_overhang);

/// @brief find the next red light stop line along the given path
/// @param [in] path predicted path to check for a stop line
/// @param [in] ego_data ego data with the stop lines information
/// @return the first red light stop line found along the path (if any)
std::optional<universe_utils::LineString2d> find_next_stop_line(
  const autoware_perception_msgs::msg::PredictedPath & path, const EgoData & ego_data);

/// @brief cut predicted path beyond stop lines of red lights
/// @param [inout] predicted_path predicted path to cut
/// @param [in] ego_data ego data with the stop lines information
void cut_predicted_path_beyond_red_lights(
  autoware_perception_msgs::msg::PredictedPath & predicted_path, const EgoData & ego_data,
  const double object_front_overhang);

/// @brief filter predicted objects and their predicted paths
/// @param [in] planner_data planner data
/// @param [in] ego_data ego data
/// @param [in] params parameters
/// @return filtered predicted objects
autoware_perception_msgs::msg::PredictedObjects filter_predicted_objects(
  const PlannerData & planner_data, const EgoData & ego_data, const PlannerParam & params);
}  // namespace autoware::motion_velocity_planner::out_of_lane

#endif  // FILTER_PREDICTED_OBJECTS_HPP_
