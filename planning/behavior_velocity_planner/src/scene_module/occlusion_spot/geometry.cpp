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
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/occlusion_spot/risk_predictive_braking.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
using lanelet::BasicLineString2d;
using lanelet::BasicPoint2d;
using lanelet::BasicPolygon2d;
namespace bg = boost::geometry;
namespace lg = lanelet::geometry;

BasicPoint2d calculateLateralOffsetPoint(
  const BasicPoint2d & p0, const BasicPoint2d & p1, const double offset)
{
  // translation
  const double dy = p1[1] - p0[1];
  const double dx = p1[0] - p0[0];
  // rotation (use inverse matrix of rotation)
  const double yaw = std::atan2(dy, dx);
  const double offset_x = p1[0] - std::sin(yaw) * offset;
  const double offset_y = p1[1] + std::cos(yaw) * offset;
  return BasicPoint2d{offset_x, offset_y};
}

void buildSlices(
  std::vector<Slice> & slices, const lanelet::ConstLanelet & path_lanelet, const double offset,
  const bool is_on_right, const PlannerParam & param)
{
  BasicLineString2d center_line = path_lanelet.centerline2d().basicLineString();
  const auto & p = param;
  /**
   * @brief relationships for interpolated polygon
   *
   * +(min_length,max_distance)-+ - +---+(max_length,max_distance) = outer_polygons
   * |                                  |
   * +--------------------------+ - +---+(max_length,min_distance) = inner_polygons
   */
  const double min_length = offset;  // + p.baselink_to_front;
  // Note: min_detection_area_length is for occlusion spot visualization but not effective for
  // planning
  const double min_detection_area_length = 10.0;
  const double max_length = std::max(
    min_detection_area_length, std::min(p.detection_area_max_length, p.detection_area_length));
  const double min_distance = (is_on_right) ? -p.half_vehicle_width : p.half_vehicle_width;
  const double slice_length = p.detection_area.slice_length;
  const int num_step = static_cast<int>(slice_length);
  //! max index is the last index of path point
  const int max_index = static_cast<int>(center_line.size() - 2);
  int idx = 0;
  /**
   * Note: create polygon from path point is better than from ego offset to consider below
   * - less computation cost and no need to recalculated interpolated point start from ego
   * - less stable for localization noise
   **/
  for (int s = 0; s < max_index; s += num_step) {
    const double length = s * slice_length;
    const double next_length = static_cast<double>(s + num_step);
    /// if (max_length < length) continue;
    Slice slice;
    BasicLineString2d inner_polygons;
    BasicLineString2d outer_polygons;
    // build connected polygon for lateral
    for (int i = 0; i <= num_step; i++) {
      idx = s + i;
      const double arc_length_from_ego = std::max(0.0, static_cast<double>(idx - min_length));
      if (arc_length_from_ego > max_length) break;
      if (idx >= max_index) break;
      const auto & c0 = center_line.at(idx);
      const auto & c1 = center_line.at(idx + 1);
      /**
       * @brief points
       * +--outer point (lateral distance obstacle can reach)
       * |
       * +--inner point(min distance)
       */
      const BasicPoint2d inner_point = calculateLateralOffsetPoint(c0, c1, min_distance);
      double lateral_distance = calculateLateralDistanceFromTTC(arc_length_from_ego, param);
      if (is_on_right) lateral_distance *= -1;
      const BasicPoint2d outer_point = calculateLateralOffsetPoint(c0, c1, lateral_distance);
      inner_polygons.emplace_back(inner_point);
      outer_polygons.emplace_back(outer_point);
    }
    if (inner_polygons.empty()) continue;
    //  connect invert point
    inner_polygons.insert(inner_polygons.end(), outer_polygons.rbegin(), outer_polygons.rend());
    slice.polygon = lanelet::BasicPolygon2d(inner_polygons);
    // add range info
    slice.range.min_length = length;
    slice.range.max_length = next_length;
    slices.emplace_back(slice);
  }
}

void buildDetectionAreaPolygon(
  std::vector<Slice> & slices, const PathWithLaneId & path, const double offset,
  const PlannerParam & param)
{
  std::vector<Slice> left_slices;
  std::vector<Slice> right_slices;
  lanelet::ConstLanelet path_lanelet = toPathLanelet(path);
  // in most case lateral distance is much more effective for velocity planning
  buildSlices(left_slices, path_lanelet, offset, false /*is_on_right*/, param);
  buildSlices(right_slices, path_lanelet, offset, true /*is_on_right*/, param);
  // Properly order lanelets from closest to furthest
  slices = left_slices;
  slices.insert(slices.end(), right_slices.begin(), right_slices.end());
  return;
}
}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner
