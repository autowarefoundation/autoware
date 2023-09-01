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

#include <Eigen/Geometry>
#include <interpolation/linear_interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <boost/geometry.hpp>

namespace drivable_area_expansion
{

std::vector<PathPointWithLaneId> crop_and_resample(
  const std::vector<PathPointWithLaneId> & points,
  const std::shared_ptr<const behavior_path_planner::PlannerData> planner_data,
  const double resample_interval)
{
  auto lon_offset = 0.0;
  auto crop_pose = *planner_data->drivable_area_expansion_prev_crop_pose;
  // reuse or update the previous crop point
  if (planner_data->drivable_area_expansion_prev_crop_pose) {
    const auto lon_offset = motion_utils::calcSignedArcLength(
      points, points.front().point.pose.position, crop_pose.position);
    if (lon_offset < 0.0) {
      planner_data->drivable_area_expansion_prev_crop_pose.reset();
    } else {
      const auto is_behind_ego =
        motion_utils::calcSignedArcLength(
          points, crop_pose.position, planner_data->self_odometry->pose.pose.position) > 0.0;
      const auto is_too_far = motion_utils::calcLateralOffset(points, crop_pose.position) > 0.1;
      if (!is_behind_ego || is_too_far)
        planner_data->drivable_area_expansion_prev_crop_pose.reset();
    }
  }
  if (!planner_data->drivable_area_expansion_prev_crop_pose) {
    crop_pose = planner_data->drivable_area_expansion_prev_crop_pose.value_or(
      motion_utils::calcInterpolatedPose(points, resample_interval - lon_offset));
  }
  // crop
  const auto crop_seg_idx = motion_utils::findNearestSegmentIndex(points, crop_pose.position);
  const auto cropped_points = motion_utils::cropPoints(
    points, crop_pose.position, crop_seg_idx + 1,
    planner_data->drivable_area_expansion_parameters.max_path_arc_length, 0.0);
  planner_data->drivable_area_expansion_prev_crop_pose = crop_pose;
  // resample
  PathWithLaneId cropped_path;
  if (tier4_autoware_utils::calcDistance2d(crop_pose, cropped_points.front()) > 1e-3) {
    PathPointWithLaneId crop_path_point;
    crop_path_point.point.pose = crop_pose;
    cropped_path.points.push_back(crop_path_point);
  }
  cropped_path.points.insert(
    cropped_path.points.end(), cropped_points.begin(), cropped_points.end());
  const auto resampled_path =
    motion_utils::resamplePath(cropped_path, resample_interval, true, true, false);

  return resampled_path.points;
}

void expandDrivableArea(
  PathWithLaneId & path,
  const std::shared_ptr<const behavior_path_planner::PlannerData> planner_data,
  const lanelet::ConstLanelets & path_lanes)
{
  const auto & params = planner_data->drivable_area_expansion_parameters;
  const auto & dynamic_objects = *planner_data->dynamic_object;
  const auto & route_handler = *planner_data->route_handler;
  const auto uncrossable_lines =
    extractUncrossableLines(*route_handler.getLaneletMapPtr(), params.avoid_linestring_types);
  multi_linestring_t uncrossable_lines_in_range;
  const auto & p = path.points.front().point.pose.position;
  for (const auto & line : uncrossable_lines)
    if (boost::geometry::distance(line, point_t{p.x, p.y}) < params.max_path_arc_length)
      uncrossable_lines_in_range.push_back(line);
  const auto points = crop_and_resample(path.points, planner_data, params.resample_interval);
  const auto path_footprints = createPathFootprints(points, params);
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
  const PathWithLaneId & path, const multi_polygon_t & expansion_polygons)
{
  polygon_t original_da_poly;
  original_da_poly.outer().reserve(path.left_bound.size() + path.right_bound.size() + 1);
  for (const auto & p : path.left_bound) original_da_poly.outer().push_back(convert_point(p));
  for (auto it = path.right_bound.rbegin(); it != path.right_bound.rend(); ++it)
    original_da_poly.outer().push_back(convert_point(*it));
  original_da_poly.outer().push_back(original_da_poly.outer().front());

  multi_polygon_t unions;
  auto expanded_da_poly = original_da_poly;
  for (const auto & p : expansion_polygons) {
    unions.clear();
    boost::geometry::union_(expanded_da_poly, p, unions);
    if (unions.size() == 1)  // union of overlapping polygons should produce a single polygon
      expanded_da_poly = unions[0];
  }
  return expanded_da_poly;
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
  const auto is_left_of_path = [&](const point_t & p) {
    return motion_utils::calcLateralOffset(path.points, convert_point(p)) > 0.0;
  };
  // prepare delimiting lines: start and end of the original expanded drivable area
  const auto start_segment =
    segment_t{convert_point(path.left_bound.front()), convert_point(path.right_bound.front())};
  const auto end_segment =
    segment_t{convert_point(path.left_bound.back()), convert_point(path.right_bound.back())};
  point_t start_segment_center;
  boost::geometry::centroid(start_segment, start_segment_center);
  const auto path_start_segment =
    segment_t{start_segment_center, convert_point(path.points[1].point.pose.position)};
  point_t end_segment_center;
  boost::geometry::centroid(end_segment, end_segment_center);
  const auto path_end_segment =
    segment_t{convert_point(path.points.back().point.pose.position), end_segment_center};
  const auto segment_to_line_intersection =
    [](const auto p1, const auto p2, const auto q1, const auto q2) -> std::optional<point_t> {
    const auto line = Eigen::Hyperplane<double, 2>::Through(q1, q2);
    const auto segment = Eigen::Hyperplane<double, 2>::Through(p1, p2);
    const auto intersection = line.intersection(segment);
    std::optional<point_t> result;
    const auto is_on_segment =
      (p1.x() <= p2.x() ? intersection.x() >= p1.x() && intersection.x() <= p2.x()
                        : intersection.x() <= p1.x() && intersection.x() >= p2.x()) &&
      (p1.y() <= p2.y() ? intersection.y() >= p1.y() && intersection.y() <= p2.y()
                        : intersection.y() <= p1.y() && intersection.y() >= p2.y());
    if (is_on_segment) result = point_t{intersection.x(), intersection.y()};
    return result;
  };
  // find intersection between the expanded drivable area and the delimiting lines
  const auto & da = expanded_drivable_area.outer();
  struct Intersection
  {
    point_t intersection_point;
    ring_t::const_iterator segment_it;
    double distance = std::numeric_limits<double>::max();
    explicit Intersection(ring_t::const_iterator it) : segment_it(it) {}
    void update(const point_t & p, const ring_t::const_iterator & it, const double dist)
    {
      intersection_point = p;
      segment_it = it;
      distance = dist;
    }
  };
  Intersection start_left(da.end());
  Intersection end_left(da.end());
  Intersection start_right(da.end());
  Intersection end_right(da.end());
  for (auto it = da.begin(); it != da.end(); ++it) {
    if (boost::geometry::distance(*it, start_segment.first) < 1e-3)
      start_left.update(*it, it, 0.0);
    else if (boost::geometry::distance(*it, start_segment.second) < 1e-3)
      start_right.update(*it, it, 0.0);
    else if (boost::geometry::distance(*it, end_segment.first) < 1e-3)
      end_left.update(*it, it, 0.0);
    else if (boost::geometry::distance(*it, end_segment.second) < 1e-3)
      end_right.update(*it, it, 0.0);
    const auto inter_start =
      std::next(it) == da.end()
        ? segment_to_line_intersection(*it, da.front(), start_segment.first, start_segment.second)
        : segment_to_line_intersection(
            *it, *std::next(it), start_segment.first, start_segment.second);
    if (inter_start) {
      const auto dist = boost::geometry::distance(*inter_start, path_start_segment);
      const auto is_left = is_left_of_path(*inter_start);
      if (is_left && dist < start_left.distance)
        start_left.update(*inter_start, it, dist);
      else if (!is_left && dist < start_right.distance)
        start_right.update(*inter_start, it, dist);
    }
    const auto inter_end =
      std::next(it) == da.end()
        ? segment_to_line_intersection(*it, da.front(), end_segment.first, end_segment.second)
        : segment_to_line_intersection(*it, *std::next(it), end_segment.first, end_segment.second);
    if (inter_end) {
      const auto dist = boost::geometry::distance(*inter_end, path_end_segment);
      const auto is_left = is_left_of_path(*inter_end);
      if (is_left && dist < end_left.distance)
        end_left.update(*inter_end, it, dist);
      else if (!is_left && dist < end_right.distance)
        end_right.update(*inter_end, it, dist);
    }
  }
  if (start_left.segment_it == da.end()) {
    const auto closest_it =
      std::min_element(da.begin(), da.end(), [&](const auto & a, const auto & b) {
        return boost::geometry::distance(a, start_segment.first) <
               boost::geometry::distance(b, start_segment.first);
      });
    start_left.update(*closest_it, closest_it, 0.0);
  }
  if (start_right.segment_it == da.end()) {
    const auto closest_it =
      std::min_element(da.begin(), da.end(), [&](const auto & a, const auto & b) {
        return boost::geometry::distance(a, start_segment.second) <
               boost::geometry::distance(b, start_segment.second);
      });
    start_right.update(*closest_it, closest_it, 0.0);
  }
  if (end_left.segment_it == da.end()) {
    const auto closest_it =
      std::min_element(da.begin(), da.end(), [&](const auto & a, const auto & b) {
        return boost::geometry::distance(a, end_segment.first) <
               boost::geometry::distance(b, end_segment.first);
      });
    end_left.update(*closest_it, closest_it, 0.0);
  }
  if (end_right.segment_it == da.end()) {
    const auto closest_it =
      std::min_element(da.begin(), da.end(), [&](const auto & a, const auto & b) {
        return boost::geometry::distance(a, end_segment.second) <
               boost::geometry::distance(b, end_segment.second);
      });
    end_right.update(*closest_it, closest_it, 0.0);
  }

  // extract the expanded left and right bound from the expanded drivable area
  path.left_bound.clear();
  path.right_bound.clear();
  path.left_bound.push_back(convert_point(start_left.intersection_point));
  path.right_bound.push_back(convert_point(start_right.intersection_point));
  if (!boost::geometry::equals(start_right.intersection_point, *start_right.segment_it))
    path.right_bound.push_back(convert_point(*start_right.segment_it));
  if (start_left.segment_it < end_left.segment_it) {
    for (auto it = std::next(start_left.segment_it); it <= end_left.segment_it; ++it)
      path.left_bound.push_back(convert_point(*it));
  } else {
    for (auto it = std::next(start_left.segment_it); it < da.end(); ++it)
      path.left_bound.push_back(convert_point(*it));
    for (auto it = da.begin(); it <= end_left.segment_it; ++it)
      path.left_bound.push_back(convert_point(*it));
  }
  if (!boost::geometry::equals(end_left.intersection_point, *end_left.segment_it))
    path.left_bound.push_back(convert_point(end_left.intersection_point));
  if (start_right.segment_it < end_right.segment_it) {
    for (auto it = std::prev(start_right.segment_it); it >= da.begin(); --it)
      path.right_bound.push_back(convert_point(*it));
    for (auto it = std::prev(da.end()); it > end_right.segment_it; --it)
      path.right_bound.push_back(convert_point(*it));
  } else {
    for (auto it = std::prev(start_right.segment_it); it > end_right.segment_it; --it)
      path.right_bound.push_back(convert_point(*it));
  }
  if (!boost::geometry::equals(end_right.intersection_point, *std::next(end_right.segment_it)))
    path.right_bound.push_back(convert_point(end_right.intersection_point));
  // remove possible duplicated points
  const auto point_cmp = [](const auto & p1, const auto & p2) {
    return p1.x == p2.x && p1.y == p2.y;
  };
  path.left_bound.erase(
    std::unique(path.left_bound.begin(), path.left_bound.end(), point_cmp), path.left_bound.end());
  path.right_bound.erase(
    std::unique(path.right_bound.begin(), path.right_bound.end(), point_cmp),
    path.right_bound.end());
  copy_z_over_arc_length(original_left_bound, path.left_bound);
  copy_z_over_arc_length(original_right_bound, path.right_bound);
}
}  // namespace drivable_area_expansion
