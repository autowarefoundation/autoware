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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__EXPANSION_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__EXPANSION_HPP_

#include "behavior_path_planner/utils/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"

#include <route_handler/route_handler.hpp>

#include <lanelet2_core/Forward.h>

#include <string>
#include <vector>

namespace drivable_area_expansion
{
/// @brief Calculate the distance limit required for the polygon to not cross the limit lines
/// @details Calculate the minimum distance from base_ls to an intersection of limit_lines and
/// expansion_polygon
/// @param[in] base_ls base linestring from which the distance is calculated
/// @param[in] expansion_polygon polygon to consider
/// @param[in] limit_lines lines we do not want to cross
/// @return distance limit
double calculateDistanceLimit(
  const linestring_t & base_ls, const polygon_t & expansion_polygon,
  const multi_linestring_t & limit_lines);

/// @brief Calculate the distance limit required for the polygon to not cross the limit polygons.
/// @details Calculate the minimum distance from base_ls to an intersection of limit_polygons and
/// expansion_polygon
/// @param[in] base_ls base linestring from which the distance is calculated
/// @param[in] expansion_polygon polygon to consider
/// @param[in] limit_polygons polygons we do not want to cross
/// @return distance limit
double calculateDistanceLimit(
  const linestring_t & base_ls, const polygon_t & expansion_polygon,
  const multi_polygon_t & limit_polygons);

/// @brief Create a polygon from a base line with a given expansion distance
/// @param[in] base_ls base linestring from which the polygon is created
/// @param[in] dist desired expansion distance from the base line
/// @param[in] is_left_side desired side of the expansion from the base line
/// @return expansion polygon
polygon_t createExpansionPolygon(
  const linestring_t & base_ls, const double dist, const bool is_left_side);

/// @brief Create polygons for the area where the drivable area should be expanded
/// @param[in] path path and its drivable area
/// @param[in] path_footprints polygons of the ego footprint projected along the path
/// @param[in] predicted_paths polygons of the dynamic objects' predicted paths
/// @param[in] uncrossable_lines lines that should not be crossed by the expanded drivable area
/// @param[in] params expansion parameters
/// @return expansion polygons
multi_polygon_t createExpansionPolygons(
  const PathWithLaneId & path, const multi_polygon_t & path_footprints,
  const multi_polygon_t & predicted_paths, const multi_linestring_t & uncrossable_lines,
  const DrivableAreaExpansionParameters & params);

/// @brief Create polygons for the area where the drivable area should be expanded
/// @param[in] path_lanes lanelets of the current path
/// @param[in] route_handler route handler
/// @param[in] path_footprints polygons of the ego footprint projected along the path
/// @param[in] predicted_paths polygons of the dynamic objects' predicted paths
/// @param[in] params expansion parameters
/// @return expansion polygons
multi_polygon_t createExpansionLaneletPolygons(
  const lanelet::ConstLanelets & path_lanes, const route_handler::RouteHandler & route_handler,
  const multi_polygon_t & path_footprints, const multi_polygon_t & predicted_paths,
  const DrivableAreaExpansionParameters & params);
}  // namespace drivable_area_expansion

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__EXPANSION_HPP_
