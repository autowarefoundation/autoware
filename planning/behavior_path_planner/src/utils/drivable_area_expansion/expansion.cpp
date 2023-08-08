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

#include "behavior_path_planner/utils/drivable_area_expansion/expansion.hpp"

#include "behavior_path_planner/utils/drivable_area_expansion/map_utils.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/path_projection.hpp"

namespace drivable_area_expansion
{

double calculateDistanceLimit(
  const linestring_t & base_ls, const polygon_t & expansion_polygon,
  const multilinestring_t & limit_lines)
{
  auto dist_limit = std::numeric_limits<double>::max();
  multipoint_t intersections;
  boost::geometry::intersection(expansion_polygon, limit_lines, intersections);
  for (const auto & p : intersections)
    dist_limit = std::min(dist_limit, boost::geometry::distance(p, base_ls));
  return dist_limit;
}

double calculateDistanceLimit(
  const linestring_t & base_ls, const polygon_t & expansion_polygon,
  const multipolygon_t & limit_polygons)
{
  auto dist_limit = std::numeric_limits<double>::max();
  for (const auto & polygon : limit_polygons) {
    multipoint_t intersections;
    boost::geometry::intersection(expansion_polygon, polygon, intersections);
    for (const auto & p : intersections)
      dist_limit = std::min(dist_limit, boost::geometry::distance(p, base_ls));
  }
  return dist_limit;
}

polygon_t createExpansionPolygon(
  const linestring_t & base_ls, const double dist, const bool is_left_side)
{
  namespace strategy = boost::geometry::strategy::buffer;
  multipolygon_t polygons;
  // set a non 0 value for the buffer as it sometimes causes no polygon to be returned by bg:buffer
  constexpr auto zero = 0.1;
  const auto left_dist = is_left_side ? dist : zero;
  const auto right_dist = !is_left_side ? dist : zero;
  const auto distance_strategy = strategy::distance_asymmetric<double>(left_dist, right_dist);
  boost::geometry::buffer(
    base_ls, polygons, distance_strategy, strategy::side_straight(), strategy::join_miter(),
    strategy::end_flat(), strategy::point_square());
  return polygons.front();
}

std::array<double, 3> calculate_arc_length_range_and_distance(
  const linestring_t & path_ls, const polygon_t & footprint, const linestring_t & bound,
  const bool is_left, const double path_length)
{
  multipoint_t intersections;
  double expansion_dist = 0.0;
  double from_arc_length = std::numeric_limits<double>::max();
  double to_arc_length = std::numeric_limits<double>::min();
  boost::geometry::intersection(footprint, bound, intersections);
  if (!intersections.empty()) {
    for (const auto & intersection : intersections) {
      const auto projection = point_to_linestring_projection(intersection, path_ls);
      if (projection.arc_length <= 0.0 || projection.arc_length >= path_length) continue;
      from_arc_length = std::min(from_arc_length, projection.arc_length);
      to_arc_length = std::max(to_arc_length, projection.arc_length);
    }
    for (const auto & p : footprint.outer()) {
      const auto projection = point_to_linestring_projection(p, path_ls);
      if (projection.arc_length <= 0.0 || projection.arc_length >= path_length) continue;
      if (is_left == (projection.distance > 0) && std::abs(projection.distance) > expansion_dist) {
        expansion_dist = std::abs(projection.distance);
        from_arc_length = std::min(from_arc_length, projection.arc_length);
        to_arc_length = std::max(to_arc_length, projection.arc_length);
      }
    }
  }
  return std::array<double, 3>({from_arc_length, to_arc_length, expansion_dist});
}

polygon_t create_compensation_polygon(
  const linestring_t & base_ls, const double compensation_dist, const bool is_left,
  const multilinestring_t uncrossable_lines, const multipolygon_t & predicted_paths)
{
  polygon_t compensation_polygon = createExpansionPolygon(base_ls, compensation_dist, !is_left);
  double dist_limit = std::min(
    compensation_dist, calculateDistanceLimit(base_ls, compensation_polygon, uncrossable_lines));
  if (!predicted_paths.empty())
    dist_limit =
      std::min(dist_limit, calculateDistanceLimit(base_ls, compensation_polygon, predicted_paths));
  if (dist_limit < compensation_dist)
    compensation_polygon = createExpansionPolygon(base_ls, dist_limit, !is_left);
  return compensation_polygon;
}

multipolygon_t createExpansionPolygons(
  const PathWithLaneId & path, const multipolygon_t & path_footprints,
  const multipolygon_t & predicted_paths, const multilinestring_t & uncrossable_lines,
  const DrivableAreaExpansionParameters & params)
{
  linestring_t path_ls;
  linestring_t left_ls;
  linestring_t right_ls;
  for (const auto & p : path.points)
    path_ls.emplace_back(p.point.pose.position.x, p.point.pose.position.y);
  for (const auto & p : path.left_bound) left_ls.emplace_back(p.x, p.y);
  for (const auto & p : path.right_bound) right_ls.emplace_back(p.x, p.y);
  const auto path_length = static_cast<double>(boost::geometry::length(path_ls));

  multipolygon_t expansion_polygons;
  for (const auto & footprint : path_footprints) {
    bool is_left = true;
    for (const auto & bound : {left_ls, right_ls}) {
      auto [from_arc_length, to_arc_length, footprint_dist] =
        calculate_arc_length_range_and_distance(path_ls, footprint, bound, is_left, path_length);
      if (footprint_dist > 0.0) {
        from_arc_length -= params.extra_arc_length;
        to_arc_length += params.extra_arc_length;
        from_arc_length = std::max(0.0, from_arc_length);
        to_arc_length = std::min(path_length, to_arc_length);
        const auto base_ls = sub_linestring(path_ls, from_arc_length, to_arc_length);
        const auto expansion_dist = params.max_expansion_distance != 0.0
                                      ? std::min(params.max_expansion_distance, footprint_dist)
                                      : footprint_dist;
        auto expansion_polygon = createExpansionPolygon(base_ls, expansion_dist, is_left);
        auto limited_dist = expansion_dist;
        const auto uncrossable_dist_limit = std::max(
          0.0, calculateDistanceLimit(base_ls, expansion_polygon, uncrossable_lines) -
                 params.avoid_linestring_dist);
        if (uncrossable_dist_limit < limited_dist) {
          limited_dist = uncrossable_dist_limit;
          if (params.compensate_uncrossable_lines) {
            const auto compensation_dist =
              footprint_dist - limited_dist + params.compensate_extra_dist;
            expansion_polygons.push_back(create_compensation_polygon(
              base_ls, compensation_dist, is_left, uncrossable_lines, predicted_paths));
          }
        }
        limited_dist = std::min(
          limited_dist, calculateDistanceLimit(base_ls, expansion_polygon, predicted_paths));
        if (limited_dist < expansion_dist)
          expansion_polygon = createExpansionPolygon(base_ls, limited_dist, is_left);
        expansion_polygons.push_back(expansion_polygon);
      }
      is_left = false;
    }
  }
  return expansion_polygons;
}

multipolygon_t createExpansionLaneletPolygons(
  const lanelet::ConstLanelets & path_lanes, const route_handler::RouteHandler & route_handler,
  const multipolygon_t & path_footprints, const multipolygon_t & predicted_paths,
  const DrivableAreaExpansionParameters & params)
{
  multipolygon_t expansion_polygons;
  lanelet::ConstLanelets candidates;
  const auto already_added = [&](const auto & ll) {
    return std::find_if(candidates.begin(), candidates.end(), [&](const auto & l) {
             return ll.id() == l.id();
           }) != candidates.end();
  };
  const auto add_if_valid = [&](const auto & ll, const auto is_left) {
    const auto bound_to_check = is_left ? ll.rightBound() : ll.leftBound();
    if (std::find_if(path_lanes.begin(), path_lanes.end(), [&](const auto & l) {
          return ll.id() == l.id();
        }) == path_lanes.end())
      if (!already_added(ll) && !hasTypes(bound_to_check, params.avoid_linestring_types))
        candidates.push_back(ll);
  };
  for (const auto & current_ll : path_lanes) {
    for (const auto & left_ll :
         route_handler.getLaneletsFromPoint(current_ll.leftBound3d().front()))
      add_if_valid(left_ll, true);
    for (const auto & left_ll : route_handler.getLaneletsFromPoint(current_ll.leftBound3d().back()))
      add_if_valid(left_ll, true);
    for (const auto & right_ll :
         route_handler.getLaneletsFromPoint(current_ll.rightBound3d().front()))
      add_if_valid(right_ll, false);
    for (const auto & right_ll :
         route_handler.getLaneletsFromPoint(current_ll.rightBound3d().back()))
      add_if_valid(right_ll, false);
  }
  for (const auto & candidate : candidates) {
    polygon_t candidate_poly;
    for (const auto & p : candidate.polygon2d()) candidate_poly.outer().emplace_back(p.x(), p.y());
    boost::geometry::correct(candidate_poly);
    if (
      !boost::geometry::overlaps(candidate_poly, predicted_paths) &&
      boost::geometry::overlaps(path_footprints, candidate_poly))
      expansion_polygons.push_back(candidate_poly);
  }
  return expansion_polygons;
}

}  // namespace drivable_area_expansion
