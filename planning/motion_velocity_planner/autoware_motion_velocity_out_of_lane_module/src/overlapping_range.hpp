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

#ifndef OVERLAPPING_RANGE_HPP_
#define OVERLAPPING_RANGE_HPP_

#include "types.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{

/// @brief calculate the overlap between the given footprint and lanelet
/// @param [in] path_footprint footprint used to calculate the overlap
/// @param [in] trajectory_lanelets trajectory lanelets used to calculate arc length along the ego
/// trajectory
/// @param [in] lanelet lanelet used to calculate the overlap
/// @return the found overlap between the footprint and the lanelet
Overlap calculate_overlap(
  const lanelet::BasicPolygon2d & trajectory_footprint,
  const lanelet::ConstLanelets & trajectory_lanelets, const lanelet::ConstLanelet & lanelet);
/// @brief calculate the overlapping ranges between the trajectory footprints and a lanelet
/// @param [in] trajectory_footprints footprints used to calculate the overlaps
/// @param [in] trajectory_lanelets trajectory lanelets used to calculate arc length along the ego
/// trajectory
/// @param [in] lanelet lanelet used to calculate the overlaps
/// @param [in] params parameters
/// @return the overlapping ranges found between the footprints and the lanelet
OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & trajectory_footprints,
  const lanelet::ConstLanelets & trajectory_lanelets, const lanelet::ConstLanelet & lanelet,
  const PlannerParam & params);
/// @brief calculate the overlapping ranges between the trajectory footprints and some lanelets
/// @param [in] trajectory_footprints footprints used to calculate the overlaps
/// @param [in] trajectory_lanelets trajectory lanelets used to calculate arc length along the ego
/// trajectory
/// @param [in] lanelets lanelets used to calculate the overlaps
/// @param [in] params parameters
/// @return the overlapping ranges found between the footprints and the lanelets, sorted by
/// increasing arc length along the trajectory
OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & trajectory_footprints,
  const lanelet::ConstLanelets & trajectory_lanelets, const lanelet::ConstLanelets & lanelets,
  const PlannerParam & params);
}  // namespace autoware::motion_velocity_planner::out_of_lane

#endif  // OVERLAPPING_RANGE_HPP_
