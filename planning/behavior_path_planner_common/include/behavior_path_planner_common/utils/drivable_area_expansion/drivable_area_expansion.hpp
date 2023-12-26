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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_

#include "behavior_path_planner_common/data_manager.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/types.hpp"

#include <route_handler/route_handler.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <vector>

namespace drivable_area_expansion
{
/// @brief Expand the drivable area based on the path curvature and the vehicle dimensions
/// @param[inout] path path whose drivable area will be expanded
/// @param[inout] planner_data planning data (params, dynamic objects, vehicle info, ...)
void expand_drivable_area(
  PathWithLaneId & path,
  const std::shared_ptr<const behavior_path_planner::PlannerData> planner_data);

/// @brief try to reuse the previous path poses and their previously calculated curvatures
/// @details poses are reused if they do not deviate too much from the current path
/// @param [in] path input path
/// @param [inout] prev_poses previous poses to reuse
/// @param [inout] prev_curvatures previous curvatures to reuse
/// @param [in] ego_point current ego point
/// @param [in] params parameters for reuse criteria and resampling interval
void reuse_previous_poses(
  const PathWithLaneId & path, std::vector<Pose> & prev_poses,
  std::vector<double> & prev_curvatures, const Point & ego_point,
  const DrivableAreaExpansionParameters & params);

/// @brief calculate the minimum lane width based on the path curvature and the vehicle dimensions
/// @cite Lim, H., Kim, C., and Jo, A., "Model Predictive Control-Based Lateral Control of
/// Autonomous Large-Size Bus on Road with Large Curvature," SAE Technical Paper 2021-01-0099, 2021,
/// https://doi.org/10.4271/2021-01-0099
/// @param [in] curvature curvature
/// @param [in] params parameters containing the vehicle dimensions
/// @return minimum lane width
double calculate_minimum_lane_width(
  const double curvature_radius, const DrivableAreaExpansionParameters & params);

/// @brief calculate mappings between path poses and the given drivable area bound
/// @param [inout] expansion expansion data to update with the mapping
/// @param [in] path_poses path poses
/// @param [in] bound drivable area bound
/// @param [in] Side left or right side
void calculate_bound_index_mappings(
  Expansion & expansion, const std::vector<Pose> & path_poses, const std::vector<Point> & bound,
  const Side side);

/// @brief apply expansion distances to all bound points within the given arc length range
/// @param [inout] expansion expansion data to update
/// @param [in] bound drivable area bound
/// @param [in] arc_length_range [m] arc length range where the expansion distances are also applied
/// @param [in] Side left or right side
void apply_arc_length_range_smoothing(
  Expansion & expansion, const std::vector<Point> & bound, const double arc_length_range,
  const Side side);

/// @brief calculate minimum lane widths and mappings between path and and drivable area bounds
/// @param [in] path_poses path poses
/// @param [in] left_bound left drivable area bound
/// @param [in] right_bound right drivable area bound
/// @param [in] curvatures curvatures at each path point
/// @param [in] params parameters with the vehicle dimensions used to calculate the min lane width
/// @return expansion data (path->bound mappings, min lane widths, ...)
Expansion calculate_expansion(
  const std::vector<Pose> & path_poses, const std::vector<Point> & left_bound,
  const std::vector<Point> & right_bound, const std::vector<double> & curvatures,
  const DrivableAreaExpansionParameters & params);

/// @brief smooth the bound by applying a limit on its rate of change
/// @details rate of change is the lateral distance from the path over the arc length along the path
/// @param [inout] bound_distances bound distances (lateral distance from the path)
/// @param [in] bound_points bound points
/// @param [in] max_rate [m/m] maximum rate of lateral deviation over arc length
void apply_bound_change_rate_limit(
  std::vector<double> & distances, const std::vector<Point> & bound, const double max_rate);

/// @brief calculate the maximum distance by which a bound can be expanded
/// @param [in] bound bound points
/// @param [in] uncrossable_segments segments that limit the bound expansion, indexed in a Rtree
/// @param [in] uncrossable_polygons polygons that limit the bound expansion
/// @param [in] params parameters with the buffer distance to keep with lines,
/// and the static maximum expansion distance
/// @param [in] Side left or right side
std::vector<double> calculate_maximum_distance(
  const std::vector<Point> & bound, const SegmentRtree & uncrossable_lines,
  const std::vector<Polygon2d> & uncrossable_polygons,
  const DrivableAreaExpansionParameters & params, const Side side);

/// @brief expand a bound by the given lateral distances away from the path
/// @param [inout] bound bound points to expand
/// @param [in] path_poses input path
/// @param [in] distances new lateral distances of the bound points away from the path
void expand_bound(
  std::vector<Point> & bound, const std::vector<Pose> & path_poses,
  const std::vector<double> & distances);

/// @brief calculate the expansion distances of the left and right drivable area bounds
/// @param [inout] expansion expansion data to be updated with the left/right expansion distances
/// @param [in] max_left_expansions maximum left expansion distances
/// @param [in] max_right_expansions maximum right expansion distances
void calculate_expansion_distances(
  Expansion & expansion, const std::vector<double> & max_left_expansions,
  const std::vector<double> & max_right_expansions);

/// @brief calculate smoothed curvatures
/// @details smoothing is done using a moving average
/// @param [in] poses input poses used to calculate the curvatures
/// @param [in] smoothing_window_size size of the window used for the moving average
/// @return smoothed curvatures of the input poses
std::vector<double> calculate_smoothed_curvatures(
  const std::vector<Pose> & poses, const size_t smoothing_window_size);

}  // namespace drivable_area_expansion

// clang-format off
#endif  // BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__DRIVABLE_AREA_EXPANSION_HPP_  // NOLINT
// clang-format on
