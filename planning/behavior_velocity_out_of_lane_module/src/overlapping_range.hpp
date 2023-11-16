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

#ifndef OVERLAPPING_RANGE_HPP_
#define OVERLAPPING_RANGE_HPP_

#include "types.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <vector>

namespace behavior_velocity_planner::out_of_lane
{

/// @brief calculate the overlap between the given footprint and lanelet
/// @param [in] path_footprint footprint used to calculate the overlap
/// @param [in] path_lanelets path lanelets used to calculate arc length along the ego path
/// @param [in] lanelet lanelet used to calculate the overlap
/// @return the found overlap between the footprint and the lanelet
Overlap calculate_overlap(
  const lanelet::BasicPolygon2d & path_footprint, const lanelet::ConstLanelets & path_lanelets,
  const lanelet::ConstLanelet & lanelet);
/// @brief calculate the overlapping ranges between the path footprints and a lanelet
/// @param [in] path_footprints footprints used to calculate the overlaps
/// @param [in] path_lanelets path lanelets used to calculate arc length along the ego path
/// @param [in] lanelet lanelet used to calculate the overlaps
/// @param [in] params parameters
/// @return the overlapping ranges found between the footprints and the lanelet
OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & path_footprints,
  const lanelet::ConstLanelets & path_lanelets, const lanelet::ConstLanelet & lanelet,
  const PlannerParam & params);
/// @brief calculate the overlapping ranges between the path footprints and some lanelets
/// @param [in] path_footprints footprints used to calculate the overlaps
/// @param [in] path_lanelets path lanelets used to calculate arc length along the ego path
/// @param [in] lanelets lanelets used to calculate the overlaps
/// @param [in] params parameters
/// @return the overlapping ranges found between the footprints and the lanelets, sorted by
/// increasing arc length along the path
OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & path_footprints,
  const lanelet::ConstLanelets & path_lanelets, const lanelet::ConstLanelets & lanelets,
  const PlannerParam & params);
}  // namespace behavior_velocity_planner::out_of_lane

#endif  // OVERLAPPING_RANGE_HPP_
