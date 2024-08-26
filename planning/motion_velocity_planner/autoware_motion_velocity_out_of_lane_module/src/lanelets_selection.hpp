// Copyright 2024 TIER IV, Inc.
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

#include <autoware/route_handler/route_handler.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>

namespace autoware::motion_velocity_planner::out_of_lane
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

/// @brief calculate lanelets crossed by the ego trajectory
/// @details calculated from the ids of the trajectory msg, the lanelets containing trajectory
/// points
/// @param [in] ego_data data about the ego vehicle
/// @param [in] route_handler route handler
/// @return lanelets crossed by the ego vehicle
lanelet::ConstLanelets calculate_trajectory_lanelets(
  const EgoData & ego_data, const std::shared_ptr<const route_handler::RouteHandler> route_handler);

/// @brief calculate lanelets that may not be crossed by the trajectory but may be overlapped during
/// a lane change
/// @param [in] trajectory_lanelets lanelets driven by the ego vehicle
/// @param [in] route_handler route handler
/// @return lanelets that may be overlapped by a lane change (and are not already in
/// trajectory_lanelets)
lanelet::ConstLanelets get_missing_lane_change_lanelets(
  const lanelet::ConstLanelets & trajectory_lanelets,
  const std::shared_ptr<const route_handler::RouteHandler> & route_handler);

/// @brief calculate lanelets that should be ignored
/// @param [in] trajectory_lanelets lanelets followed by the ego vehicle
/// @param [in] route_handler route handler
/// @return lanelets to ignore
lanelet::ConstLanelets calculate_ignored_lanelets(
  const lanelet::ConstLanelets & trajectory_lanelets,
  const std::shared_ptr<const route_handler::RouteHandler> & route_handler);

/// @brief calculate the polygons representing the ego lane and add it to the ego data
/// @param [inout] ego_data ego data
/// @param [in] route_handler route handler with map information
void calculate_drivable_lane_polygons(
  EgoData & ego_data, const route_handler::RouteHandler & route_handler);

/// @brief calculate the lanelets overlapped at each out of lane point
/// @param [out] out_of_lane_data data with the out of lane points
/// @param [in] route_handler route handler with the lanelet map
void calculate_overlapped_lanelets(
  OutOfLaneData & out_of_lane_data, const route_handler::RouteHandler & route_handler);
}  // namespace autoware::motion_velocity_planner::out_of_lane

#endif  // LANELETS_SELECTION_HPP_
