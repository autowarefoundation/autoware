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

#include "behavior_path_planner/utils/drivable_area_expansion/drivable_area_expansion.hpp"

#include "behavior_path_planner/utils/drivable_area_expansion/expansion.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/footprints.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/map_utils.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"
#include "interpolation/linear_interpolation.hpp"

#include <boost/geometry.hpp>

namespace drivable_area_expansion
{

void expandDrivableArea(
  PathWithLaneId & path, const DrivableAreaExpansionParameters & params,
  const PredictedObjects & dynamic_objects, const route_handler::RouteHandler & route_handler,
  const lanelet::ConstLanelets & path_lanes)
{
  const auto uncrossable_lines =
    extractUncrossableLines(*route_handler.getLaneletMapPtr(), params.avoid_linestring_types);
  multilinestring_t uncrossable_lines_in_range;
  const auto & p = path.points.front().point.pose.position;
  for (const auto & line : uncrossable_lines)
    if (boost::geometry::distance(line, point_t{p.x, p.y}) < params.max_path_arc_length)
      uncrossable_lines_in_range.push_back(line);
  const auto path_footprints = createPathFootprints(path, params);
  const auto predicted_paths = createObjectFootprints(dynamic_objects, params);
  const auto expansion_polygons =
    params.expansion_method == "lanelet"
      ? createExpansionLaneletPolygons(
          path_lanes, route_handler, path_footprints, predicted_paths, params)
      : createExpansionPolygons(
          path, path_footprints, predicted_paths, uncrossable_lines_in_range, params);
  const auto expanded_drivable_area = createExpandedDrivableAreaPolygon(path, expansion_polygons);
  updateDrivableAreaBounds(path, expanded_drivable_area);
}

point_t convert_point(const Point & p)
{
  return point_t{p.x, p.y};
}

Point convert_point(const point_t & p)
{
  return Point().set__x(p.x()).set__y(p.y());
}

polygon_t createExpandedDrivableAreaPolygon(
  const PathWithLaneId & path, const multipolygon_t & expansion_polygons)
{
  polygon_t original_da_poly;
  original_da_poly.outer().reserve(path.left_bound.size() + path.right_bound.size() + 1);
  for (const auto & p : path.left_bound) original_da_poly.outer().push_back(convert_point(p));
  for (auto it = path.right_bound.rbegin(); it != path.right_bound.rend(); ++it)
    original_da_poly.outer().push_back(convert_point(*it));
  original_da_poly.outer().push_back(original_da_poly.outer().front());

  multipolygon_t unions;
  auto expanded_da_poly = original_da_poly;
  for (const auto & p : expansion_polygons) {
    unions.clear();
    boost::geometry::union_(expanded_da_poly, p, unions);
    if (unions.size() == 1)  // union of overlapping polygons should produce a single polygon
      expanded_da_poly = unions[0];
  }
  return expanded_da_poly;
}

std::array<ring_t::const_iterator, 4> findLeftRightRanges(
  const PathWithLaneId & path, const ring_t & expanded_drivable_area)
{
  const auto is_left_of_segment = [](const point_t & a, const point_t & b, const point_t & p) {
    return (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x()) > 0;
  };
  const auto is_left_of_path_start = [&](const point_t & p) {
    return is_left_of_segment(
      convert_point(path.points[0].point.pose.position),
      convert_point(path.points[1].point.pose.position), p);
  };
  const auto is_left_of_path_end = [&, size = path.points.size()](const point_t & p) {
    return is_left_of_segment(
      convert_point(path.points[size - 2].point.pose.position),
      convert_point(path.points[size - 1].point.pose.position), p);
  };
  const auto dist_to_path_start = [start = convert_point(path.points.front().point.pose.position)](
                                    const auto & p) { return boost::geometry::distance(start, p); };
  const auto dist_to_path_end = [end = convert_point(path.points.back().point.pose.position)](
                                  const auto & p) { return boost::geometry::distance(end, p); };

  double min_start_dist = std::numeric_limits<double>::max();
  auto start_transition = expanded_drivable_area.end();
  double min_end_dist = std::numeric_limits<double>::max();
  auto end_transition = expanded_drivable_area.end();
  for (auto it = expanded_drivable_area.begin(); std::next(it) != expanded_drivable_area.end();
       ++it) {
    if (is_left_of_path_start(*it) != is_left_of_path_start(*std::next(it))) {
      const auto dist = dist_to_path_start(*it);
      if (dist < min_start_dist) {
        start_transition = it;
        min_start_dist = dist;
      }
    }
    if (is_left_of_path_end(*it) != is_left_of_path_end(*std::next(it))) {
      const auto dist = dist_to_path_end(*it);
      if (dist < min_end_dist) {
        end_transition = it;
        min_end_dist = dist;
      }
    }
  }
  const auto left_start =
    is_left_of_path_start(*start_transition) ? start_transition : std::next(start_transition);
  const auto right_start =
    is_left_of_path_start(*start_transition) ? std::next(start_transition) : start_transition;
  const auto left_end =
    is_left_of_path_end(*end_transition) ? end_transition : std::next(end_transition);
  const auto right_end =
    is_left_of_path_end(*end_transition) ? std::next(end_transition) : end_transition;
  return {left_start, left_end, right_start, right_end};
}

void copy_z_over_arc_length(
  const std::vector<geometry_msgs::msg::Point> & from, std::vector<geometry_msgs::msg::Point> & to)
{
  if (from.empty() || to.empty()) return;
  to.front().z = from.front().z;
  if (from.size() < 2 || to.size() < 2) return;
  to.back().z = from.back().z;
  auto i_from = 1lu;
  auto s_from = tier4_autoware_utils::calcDistance2d(from[0], from[1]);
  auto s_to = 0.0;
  auto s_from_prev = 0.0;
  for (auto i_to = 1lu; i_to + 1 < to.size(); ++i_to) {
    s_to += tier4_autoware_utils::calcDistance2d(to[i_to - 1], to[i_to]);
    for (; s_from < s_to && i_from + 1 < from.size(); ++i_from) {
      s_from_prev = s_from;
      s_from += tier4_autoware_utils::calcDistance2d(from[i_from], from[i_from + 1]);
    }
    if (s_from - s_from_prev != 0.0) {
      const auto ratio = (s_to - s_from_prev) / (s_from - s_from_prev);
      to[i_to].z = interpolation::lerp(from[i_from - 1].z, from[i_from].z, ratio);
    } else {
      to[i_to].z = to[i_to - 1].z;
    }
  }
}

void updateDrivableAreaBounds(PathWithLaneId & path, const polygon_t & expanded_drivable_area)
{
  const auto original_left_bound = path.left_bound;
  const auto original_right_bound = path.right_bound;
  path.left_bound.clear();
  path.right_bound.clear();
  const auto begin = expanded_drivable_area.outer().begin();
  const auto end = std::prev(expanded_drivable_area.outer().end());
  const auto & [left_start, left_end, right_start, right_end] =
    findLeftRightRanges(path, expanded_drivable_area.outer());
  // NOTE: clockwise ordering -> positive increment for left bound, negative for right bound
  if (left_start < left_end) {
    path.left_bound.reserve(std::distance(left_start, left_end));
    for (auto it = left_start; it <= left_end; ++it) path.left_bound.push_back(convert_point(*it));
  } else {  // loop back
    path.left_bound.reserve(std::distance(left_start, end) + std::distance(begin, left_end));
    for (auto it = left_start; it != end; ++it) path.left_bound.push_back(convert_point(*it));
    for (auto it = begin; it <= left_end; ++it) path.left_bound.push_back(convert_point(*it));
  }
  if (right_start > right_end) {
    path.right_bound.reserve(std::distance(right_end, right_start));
    for (auto it = right_start; it >= right_end; --it)
      path.right_bound.push_back(convert_point(*it));
  } else {  // loop back
    path.right_bound.reserve(std::distance(begin, right_start) + std::distance(right_end, end));
    for (auto it = right_start; it >= begin; --it) path.right_bound.push_back(convert_point(*it));
    for (auto it = end - 1; it >= right_end; --it) path.right_bound.push_back(convert_point(*it));
  }
  copy_z_over_arc_length(original_left_bound, path.left_bound);
  copy_z_over_arc_length(original_right_bound, path.right_bound);
}

}  // namespace drivable_area_expansion
