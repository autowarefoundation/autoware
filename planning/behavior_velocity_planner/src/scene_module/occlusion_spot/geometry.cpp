// Copyright 2021 Tier IV, Inc.
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

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/occlusion_spot/geometry.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace geometry
{
using lanelet::BasicLineString2d;
using lanelet::BasicPoint2d;
using lanelet::BasicPolygon2d;
namespace bg = boost::geometry;
namespace lg = lanelet::geometry;

void createOffsetLineString(
  const BasicLineString2d & in, const double offset, BasicLineString2d & offset_line_string)
{
  for (size_t i = 0; i < in.size() - 1; i++) {
    const auto & p0 = in.at(i);
    const auto & p1 = in.at(i + 1);
    // translation
    const double dy = p1[1] - p0[1];
    const double dx = p1[0] - p0[0];
    // rotation (use inverse matrix of rotation)
    const double yaw = std::atan2(dy, dx);
    // translation
    const double offset_x = p0[0] - std::sin(yaw) * offset;
    const double offset_y = p0[1] + std::cos(yaw) * offset;
    offset_line_string.emplace_back(BasicPoint2d{offset_x, offset_y});
    //! insert final offset linestring using prev vertical direction
    if (i == in.size() - 2) {
      const double offset_x = p1[0] - std::sin(yaw) * offset;
      const double offset_y = p1[1] + std::cos(yaw) * offset;
      offset_line_string.emplace_back(BasicPoint2d{offset_x, offset_y});
    }
  }
  return;
}

void buildSlices(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet, const SliceRange & range,
  const double slice_length, const double slice_width, const double resolution)
{
  const int num_lateral_slice =
    static_cast<int>(std::abs(range.max_distance - range.min_distance) / slice_width);
  /**
   * @brief bounds
   * +---------- outer bounds
   * |   +------ inner bounds(original path)
   * |   |
   */
  BasicLineString2d inner_bounds = path_lanelet.centerline2d().basicLineString();
  BasicLineString2d outer_bounds;
  if (inner_bounds.size() < 2) return;
  createOffsetLineString(inner_bounds, range.max_distance, outer_bounds);
  const double ratio_dist_start = std::abs(range.min_distance / range.max_distance);
  const double ratio_dist_increment = std::min(1.0, slice_width / std::abs(range.max_distance));
  lanelet::BasicPolygon2d poly;
  const int num_step = static_cast<int>(slice_length / resolution);
  //! max index is the last index of path point
  const int max_index = static_cast<int>(inner_bounds.size() - 1);
  for (int s = 0; s < max_index; s += num_step) {
    const double length = s * slice_length;
    const double next_length = (s + num_step) * resolution;
    for (int d = 0; d < num_lateral_slice; d++) {
      const double ratio_dist = ratio_dist_start + d * ratio_dist_increment;
      const double next_ratio_dist = ratio_dist_start + (d + 1.0) * ratio_dist_increment;
      Slice slice;
      BasicLineString2d inner_polygons;
      BasicLineString2d outer_polygons;
      // build interpolated polygon for lateral
      for (int i = 0; i <= num_step; i++) {
        if (s + i >= max_index) continue;
        inner_polygons.emplace_back(
          lerp(inner_bounds.at(s + i), outer_bounds.at(s + i), ratio_dist));
        outer_polygons.emplace_back(
          lerp(inner_bounds.at(s + i), outer_bounds.at(s + i), next_ratio_dist));
      }
      // Build polygon
      inner_polygons.insert(inner_polygons.end(), outer_polygons.rbegin(), outer_polygons.rend());
      slice.polygon = lanelet::BasicPolygon2d(inner_polygons);
      // add range info
      slice.range.min_length = length;
      slice.range.max_length = next_length;
      slice.range.min_distance = ratio_dist * range.max_distance;
      slice.range.max_distance = next_ratio_dist * range.max_distance;
      slices.emplace_back(slice);
    }
  }
}

void buildSidewalkSlices(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet,
  const double longitudinal_offset, const double lateral_offset, const double slice_size,
  const double lateral_max_dist)
{
  std::vector<Slice> left_slices;
  std::vector<Slice> right_slices;
  const double longitudinal_max_dist = lg::length2d(path_lanelet);
  SliceRange left_slice_range = {
    longitudinal_offset, longitudinal_max_dist, lateral_offset, lateral_offset + lateral_max_dist};
  // in most case lateral distance is much more effective for velocity planning
  const double slice_length = 4.0 * slice_size;
  const double slice_width = slice_size;
  const double resolution = 1.0;
  buildSlices(left_slices, path_lanelet, left_slice_range, slice_length, slice_width, resolution);
  SliceRange right_slice_range = {
    longitudinal_offset, longitudinal_max_dist, -lateral_offset,
    -lateral_offset - lateral_max_dist};
  buildSlices(right_slices, path_lanelet, right_slice_range, slice_length, slice_width, resolution);
  // Properly order lanelets from closest to furthest
  slices = left_slices;
  slices.insert(slices.end(), right_slices.begin(), right_slices.end());
  return;
}
}  // namespace geometry
}  // namespace behavior_velocity_planner
