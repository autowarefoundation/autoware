// Copyright 2023 TIER IV, Inc.
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

#ifndef LANELETS_SELECTION_HPP_
#define LANELETS_SELECTION_HPP_

#include "types.hpp"

#include <route_handler/route_handler.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace behavior_velocity_planner::out_of_lane
{
/// @brief checks if a lanelet is already contained in a vector of lanelets
/// @param [in] lanelets vector to check
/// @param [in] id lanelet id to check
/// @return true if the given vector contains a lanelet of the given id
inline bool contains_lanelet(const lanelet::ConstLanelets & lanelets, const lanelet::Id id)
{
  return std::find_if(lanelets.begin(), lanelets.end(), [&](const auto & l) {
           return l.id() == id;
         }) != lanelets.end();
};
/// @brief calculate lanelets that should be ignored
/// @param [in] ego_data data about the ego vehicle
/// @param [in] path_lanelets lanelets driven by the ego vehicle
/// @param [in] route_handler route handler
/// @param [in] params parameters
/// @return lanelets to ignore
lanelet::ConstLanelets calculate_ignored_lanelets(
  const EgoData & ego_data, const lanelet::ConstLanelets & path_lanelets,
  const route_handler::RouteHandler & route_handler, const PlannerParam & params);
/// @brief calculate lanelets that should be checked by the module
/// @param [in] ego_data data about the ego vehicle
/// @param [in] path_lanelets lanelets driven by the ego vehicle
/// @param [in] ignored_lanelets lanelets to ignore
/// @param [in] route_handler route handler
/// @param [in] params parameters
/// @return lanelets to check for overlaps
lanelet::ConstLanelets calculate_other_lanelets(
  const EgoData & ego_data, const lanelet::ConstLanelets & path_lanelets,
  const lanelet::ConstLanelets & ignored_lanelets,
  const route_handler::RouteHandler & route_handler, const PlannerParam & params);
}  // namespace behavior_velocity_planner::out_of_lane

#endif  // LANELETS_SELECTION_HPP_
