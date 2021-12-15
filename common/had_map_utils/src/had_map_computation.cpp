// Copyright 2021 TierIV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <CGAL/Boolean_set_operations_2.h>

#include <geometry_msgs/msg/polygon.hpp>

#include <vector>

#include "had_map_utils/had_map_computation.hpp"
#include "had_map_utils/had_map_visualization.hpp"

namespace autoware
{
namespace common
{
namespace had_map_utils
{

using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using CGAL_Point = Kernel::Point_2;
using CGAL_Polygon = CGAL::Polygon_2<Kernel>;
using CGAL_Polygon_with_holes = CGAL::Polygon_with_holes_2<Kernel>;

// TODO(s.me) this is getting a bit long, break up
lanelet::Polygon3d coalesce_drivable_areas(
  const autoware_auto_planning_msgs::msg::HADMapRoute & had_map_route,
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  CGAL_Polygon_with_holes drivable_area;

  for (const auto & map_segment : had_map_route.segments) {
    // Attempt to obtain a polygon from the primitive ID
    geometry_msgs::msg::Polygon current_area_polygon{};
    const auto & lanelet_layer = lanelet_map_ptr->laneletLayer;
    const auto & current_lanelet_candidate = lanelet_layer.find(map_segment.preferred_primitive_id);
    if (current_lanelet_candidate != lanelet_layer.end()) {
      current_area_polygon = lanelet2Polygon(*current_lanelet_candidate);
    } else {
      const auto & area_layer = lanelet_map_ptr->areaLayer;
      const auto & current_area_candidate = area_layer.find(map_segment.preferred_primitive_id);
      if (current_area_candidate != area_layer.end()) {
        current_area_polygon = area2Polygon(*current_area_candidate);
      } else {
        // This might happen if a primitive is on the route, but outside of the bounding box that we
        // query the map for. Not sure how to deal with this at this point though.
        std::cerr << "Error: primitive ID " << map_segment.preferred_primitive_id <<
          " not found, skipping" <<
          std::endl;
        continue;
      }
    }

    if (drivable_area.outer_boundary().size() > 0) {
      // Convert current_area_polygon to a CGAL_Polygon and make sure the orientation is correct
      CGAL_Polygon to_join{};
      CGAL_Polygon_with_holes temporary_union;
      const auto first_point = current_area_polygon.points.begin();
      for (auto area_point_it =
        current_area_polygon.points.begin();
        // Stop if we run out of points, or if we encounter the first point again
        area_point_it < current_area_polygon.points.end() &&
        !(first_point != area_point_it && first_point->x == area_point_it->x &&
        first_point->y == area_point_it->y);
        area_point_it++)
      {
        to_join.push_back(CGAL_Point(area_point_it->x, area_point_it->y));
      }

      if (to_join.is_clockwise_oriented() ) {
        to_join.reverse_orientation();
      }

      // Merge this CGAL polygon with the growing drivable_area. We need an intermediate merge
      // result because as far as I can tell from the CGAL docs, I can't "join to" a polygon
      // in-place with the join() interface.
      const auto polygons_overlap = CGAL::join(drivable_area, to_join, temporary_union);
      if (!polygons_overlap && !drivable_area.outer_boundary().is_empty()) {
        // TODO(s.me) cancel here? Right now we just ignore that polygon, if it doesn't
        // overlap with the rest, there is no way to get to it anyway
        std::cerr << "Error: polygons in union do not overlap!" << std::endl;
      } else {
        drivable_area = temporary_union;
      }
    } else {
      // Otherwise, just set the current drivable area equal to the area to add to it, because
      // CGAL seems to do "union(empty, non-empty) = empty" for some reason.
      const auto first_point = current_area_polygon.points.begin();
      for (auto area_point_it =
        current_area_polygon.points.begin();
        area_point_it < current_area_polygon.points.end() &&
        // Stop if we run out of points, or if we encounter the first point again
        !(first_point != area_point_it && first_point->x == area_point_it->x &&
        first_point->y == area_point_it->y);
        area_point_it++)
      {
        drivable_area.outer_boundary().push_back(CGAL_Point(area_point_it->x, area_point_it->y));
      }
      if (drivable_area.outer_boundary().is_clockwise_oriented() ) {
        drivable_area.outer_boundary().reverse_orientation();
      }
    }
  }

  // At this point, all the polygons from the route should be merged into drivable_area,
  // and we now need to turn this back into a lanelet polygon.
  std::vector<lanelet::Point3d> lanelet_drivable_area_points{};
  lanelet_drivable_area_points.reserve(drivable_area.outer_boundary().size());
  for (auto p = drivable_area.outer_boundary().vertices_begin();
    p != drivable_area.outer_boundary().vertices_end(); p++)
  {
    lanelet_drivable_area_points.emplace_back(
      lanelet::Point3d(
        lanelet::utils::getId(), CGAL::to_double(p->x()),
        CGAL::to_double(p->y()), 0.0));
  }
  lanelet::Polygon3d lanelet_drivable_area(lanelet::utils::getId(), lanelet_drivable_area_points);
  return lanelet_drivable_area;
}


}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware
