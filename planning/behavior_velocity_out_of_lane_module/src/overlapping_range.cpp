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

#include "overlapping_range.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <tf2/utils.h>

#include <algorithm>

namespace behavior_velocity_planner::out_of_lane
{

Overlap calculate_overlap(
  const lanelet::BasicPolygon2d & path_footprint, const lanelet::ConstLanelets & path_lanelets,
  const lanelet::ConstLanelet & lanelet)
{
  Overlap overlap;
  const auto & left_bound = lanelet.leftBound2d().basicLineString();
  const auto & right_bound = lanelet.rightBound2d().basicLineString();
  const auto overlap_left = boost::geometry::intersects(path_footprint, left_bound);
  const auto overlap_right = boost::geometry::intersects(path_footprint, right_bound);

  lanelet::BasicPolygons2d overlapping_polygons;
  if (overlap_left || overlap_right)
    boost::geometry::intersection(
      path_footprint, lanelet.polygon2d().basicPolygon(), overlapping_polygons);
  for (const auto & overlapping_polygon : overlapping_polygons) {
    for (const auto & point : overlapping_polygon) {
      if (overlap_left && overlap_right)
        overlap.inside_distance = boost::geometry::distance(left_bound, right_bound);
      else if (overlap_left)
        overlap.inside_distance =
          std::max(overlap.inside_distance, boost::geometry::distance(point, left_bound));
      else if (overlap_right)
        overlap.inside_distance =
          std::max(overlap.inside_distance, boost::geometry::distance(point, right_bound));
      geometry_msgs::msg::Pose p;
      p.position.x = point.x();
      p.position.y = point.y();
      const auto length = lanelet::utils::getArcCoordinates(path_lanelets, p).length;
      if (length > overlap.max_arc_length) {
        overlap.max_arc_length = length;
        overlap.max_overlap_point = point;
      }
      if (length < overlap.min_arc_length) {
        overlap.min_arc_length = length;
        overlap.min_overlap_point = point;
      }
    }
  }
  return overlap;
}

OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & path_footprints,
  const lanelet::ConstLanelets & path_lanelets, const lanelet::ConstLanelet & lanelet,
  const PlannerParam & params)
{
  OverlapRanges ranges;
  OtherLane other_lane(lanelet);
  std::vector<Overlap> overlaps;
  for (auto i = 0UL; i < path_footprints.size(); ++i) {
    const auto overlap = calculate_overlap(path_footprints[i], path_lanelets, lanelet);
    const auto has_overlap = overlap.inside_distance > params.overlap_min_dist;
    if (has_overlap) {  // open/update the range
      overlaps.push_back(overlap);
      if (!other_lane.range_is_open) {
        other_lane.first_range_bound.index = i;
        other_lane.first_range_bound.point = overlap.min_overlap_point;
        other_lane.first_range_bound.arc_length =
          overlap.min_arc_length - params.overlap_extra_length;
        other_lane.first_range_bound.inside_distance = overlap.inside_distance;
        other_lane.range_is_open = true;
      }
      other_lane.last_range_bound.index = i;
      other_lane.last_range_bound.point = overlap.max_overlap_point;
      other_lane.last_range_bound.arc_length = overlap.max_arc_length + params.overlap_extra_length;
      other_lane.last_range_bound.inside_distance = overlap.inside_distance;
    } else if (other_lane.range_is_open) {  // !has_overlap: close the range if it is open
      ranges.push_back(other_lane.close_range());
      ranges.back().debug.overlaps = overlaps;
      overlaps.clear();
    }
  }
  // close the range if it is still open
  if (other_lane.range_is_open) {
    ranges.push_back(other_lane.close_range());
    ranges.back().debug.overlaps = overlaps;
    overlaps.clear();
  }
  return ranges;
}

OverlapRanges calculate_overlapping_ranges(
  const std::vector<lanelet::BasicPolygon2d> & path_footprints,
  const lanelet::ConstLanelets & path_lanelets, const lanelet::ConstLanelets & lanelets,
  const PlannerParam & params)
{
  OverlapRanges ranges;
  for (auto & lanelet : lanelets) {
    const auto lanelet_ranges =
      calculate_overlapping_ranges(path_footprints, path_lanelets, lanelet, params);
    ranges.insert(ranges.end(), lanelet_ranges.begin(), lanelet_ranges.end());
  }
  return ranges;
}

}  // namespace behavior_velocity_planner::out_of_lane
