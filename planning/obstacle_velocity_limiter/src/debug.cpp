// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_velocity_limiter/debug.hpp"

#include "obstacle_velocity_limiter/parameters.hpp"
#include "obstacle_velocity_limiter/types.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

namespace obstacle_velocity_limiter
{
visualization_msgs::msg::Marker makeLinestringMarker(const linestring_t & ls, const Float z)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.header.frame_id = "map";
  for (const auto & point : ls) {
    geometry_msgs::msg::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = z;
    marker.points.push_back(p);
  }
  return marker;
}

visualization_msgs::msg::Marker makePolygonMarker(const polygon_t & polygon, const Float z)
{
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.header.frame_id = "map";
  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  geometry_msgs::msg::Point point;
  point.z = z;
  for (const auto & p : polygon.outer()) {
    point.x = p.x();
    point.y = p.y();
    marker.points.push_back(point);
  }
  return marker;
}

visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const Obstacles & obstacles, const std::vector<multilinestring_t> & original_projections,
  const std::vector<multilinestring_t> & adjusted_projections,
  const std::vector<polygon_t> & original_footprints,
  const std::vector<polygon_t> & adjusted_footprints, const ObstacleMasks & obstacle_masks,
  const Float marker_z)
{
  visualization_msgs::msg::MarkerArray debug_markers;
  auto original_projections_id_offset = 0;
  auto adjusted_projections_id_offset = 0;
  for (auto i = 0ul; i < original_projections.size(); ++i) {
    for (auto j = 0ul; j < original_projections[i].size(); ++j) {
      auto marker = makeLinestringMarker(original_projections[i][j], marker_z);
      marker.ns = "original_projections";
      marker.id = i + original_projections_id_offset + j;
      marker.color.r = 0.7;
      marker.color.b = 0.2;
      debug_markers.markers.push_back(marker);
    }
    original_projections_id_offset += original_projections[i].size() - 1;
    for (auto j = 0ul; j < adjusted_projections[i].size(); ++j) {
      auto marker = makeLinestringMarker(adjusted_projections[i][j], marker_z);
      marker.ns = "adjusted_projections";
      marker.id = i + adjusted_projections_id_offset + j;
      marker.color.g = 0.7;
      marker.color.b = 0.2;
      debug_markers.markers.push_back(marker);
    }
    adjusted_projections_id_offset += adjusted_projections[i].size() - 1;
    {
      auto marker = makePolygonMarker(original_footprints[i], marker_z);
      marker.ns = "original_footprints";
      marker.id = i;
      marker.color.r = 0.7;
      debug_markers.markers.push_back(marker);
    }
    {
      auto marker = makePolygonMarker(adjusted_footprints[i], marker_z);
      marker.ns = "adjusted_footprints";
      marker.id = i;
      marker.color.g = 0.7;
      debug_markers.markers.push_back(marker);
    }
  }
  const auto max_id = original_projections.size() +
                      std::max(original_projections_id_offset, adjusted_projections_id_offset);
  auto obs_id = 0lu;
  for (const auto & ls : obstacles.lines) {
    auto marker = makeLinestringMarker(ls, marker_z);
    marker.ns = "obstacles";
    marker.id = obs_id++;
    marker.color.b = 1.0;
    debug_markers.markers.push_back(marker);
  }
  visualization_msgs::msg::Marker points_marker;
  points_marker.type = visualization_msgs::msg::Marker::POINTS;
  points_marker.header.frame_id = "map";
  points_marker.scale.x = 0.1;
  points_marker.scale.y = 0.1;
  points_marker.color.a = 1.0;
  points_marker.color.b = 1.0;
  points_marker.ns = "obstacles";
  points_marker.id = obs_id++;
  geometry_msgs::msg::Point point;
  point.z = marker_z;
  for (const auto & p : obstacles.points) {
    point.x = p.x();
    point.y = p.y();
    points_marker.points.push_back(point);
  }
  debug_markers.markers.push_back(points_marker);

  // Obstacle Masks
  auto mask_id = 0lu;
  for (const auto & negative_mask : obstacle_masks.negative_masks) {
    auto marker = makePolygonMarker(negative_mask, marker_z);
    marker.ns = "obstacle_masks";
    marker.id = mask_id++;
    marker.color.r = 0.8;
    debug_markers.markers.push_back(marker);
  }
  {
    auto marker = makePolygonMarker(obstacle_masks.positive_mask, marker_z);
    marker.ns = "obstacle_masks";
    marker.id = mask_id++;
    marker.color.g = 0.8;
    debug_markers.markers.push_back(marker);
  }

  static auto prev_max_id = 0lu;
  static auto prev_size = 0lu;
  static auto prev_max_obs_id = 0lu;
  static auto prev_max_mask_id = 0lu;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETE;
  marker.ns = "obstacles";
  for (auto delete_id = obs_id; delete_id < prev_max_obs_id; ++delete_id) {
    marker.id = delete_id;
    debug_markers.markers.push_back(marker);
  }
  marker.ns = "obstacle_masks";
  for (auto delete_id = mask_id; delete_id < prev_max_mask_id; ++delete_id) {
    marker.id = delete_id;
    debug_markers.markers.push_back(marker);
  }
  for (const auto & ns : {"original_projections", "adjusted_projections"}) {
    marker.ns = ns;
    for (auto delete_id = max_id; delete_id < prev_max_id; ++delete_id) {
      marker.id = delete_id;
      debug_markers.markers.push_back(marker);
    }
  }
  for (const auto & ns : {"original_footprints", "adjusted_footprints"}) {
    marker.ns = ns;
    for (auto delete_id = original_projections.size(); delete_id < prev_size; ++delete_id) {
      marker.id = delete_id;
      debug_markers.markers.push_back(marker);
    }
  }

  prev_max_id = max_id;
  prev_size = original_projections.size();
  prev_max_obs_id = obs_id;
  prev_max_mask_id = mask_id;
  return debug_markers;
}

}  // namespace obstacle_velocity_limiter
