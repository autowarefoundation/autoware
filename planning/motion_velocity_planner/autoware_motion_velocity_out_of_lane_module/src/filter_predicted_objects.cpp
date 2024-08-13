// Copyright 2024 TIER IV, Inc.
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

#include "filter_predicted_objects.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <traffic_light_utils/traffic_light_utils.hpp>

#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/index/predicates.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <vector>

namespace autoware::motion_velocity_planner::out_of_lane
{
void cut_predicted_path_beyond_line(
  autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const universe_utils::LineString2d & stop_line, const double object_front_overhang)
{
  if (predicted_path.path.empty() || stop_line.size() < 2) return;

  auto stop_line_idx = 0UL;
  bool found = false;
  lanelet::BasicSegment2d path_segment;
  path_segment.first.x() = predicted_path.path.front().position.x;
  path_segment.first.y() = predicted_path.path.front().position.y;
  for (stop_line_idx = 1; stop_line_idx < predicted_path.path.size(); ++stop_line_idx) {
    path_segment.second.x() = predicted_path.path[stop_line_idx].position.x;
    path_segment.second.y() = predicted_path.path[stop_line_idx].position.y;
    if (boost::geometry::intersects(stop_line, path_segment)) {
      found = true;
      break;
    }
    path_segment.first = path_segment.second;
  }
  if (found) {
    auto cut_idx = stop_line_idx;
    double arc_length = 0;
    while (cut_idx > 0 && arc_length < object_front_overhang) {
      arc_length += autoware::universe_utils::calcDistance2d(
        predicted_path.path[cut_idx], predicted_path.path[cut_idx - 1]);
      --cut_idx;
    }
    predicted_path.path.resize(cut_idx);
  }
}

std::optional<universe_utils::LineString2d> find_next_stop_line(
  const autoware_perception_msgs::msg::PredictedPath & path, const EgoData & ego_data)
{
  universe_utils::LineString2d query_path;
  for (const auto & p : path.path) query_path.emplace_back(p.position.x, p.position.y);
  std::vector<StopLineNode> query_results;
  ego_data.stop_lines_rtree.query(
    boost::geometry::index::intersects(query_path), std::back_inserter(query_results));
  auto earliest_intersecting_index = query_path.size();
  std::optional<universe_utils::LineString2d> earliest_stop_line;
  universe_utils::Segment2d path_segment;
  for (const auto & [_, stop_line] : query_results) {
    for (auto index = 0UL; index + 1 < query_path.size(); ++index) {
      path_segment.first = query_path[index];
      path_segment.second = query_path[index + 1];
      if (boost::geometry::intersects(path_segment, stop_line.stop_line)) {
        bool within_stop_line_lanelet =
          std::any_of(stop_line.lanelets.begin(), stop_line.lanelets.end(), [&](const auto & ll) {
            return boost::geometry::within(path_segment.first, ll.polygon2d().basicPolygon());
          });
        if (within_stop_line_lanelet) {
          earliest_intersecting_index = std::min(index, earliest_intersecting_index);
          earliest_stop_line = stop_line.stop_line;
        }
      }
    }
  }
  return earliest_stop_line;
}

void cut_predicted_path_beyond_red_lights(
  autoware_perception_msgs::msg::PredictedPath & predicted_path, const EgoData & ego_data,
  const double object_front_overhang)
{
  const auto stop_line = find_next_stop_line(predicted_path, ego_data);
  if (stop_line) {
    cut_predicted_path_beyond_line(predicted_path, *stop_line, object_front_overhang);
  }
}

autoware_perception_msgs::msg::PredictedObjects filter_predicted_objects(
  const std::shared_ptr<const PlannerData> planner_data, const EgoData & ego_data,
  const PlannerParam & params)
{
  autoware_perception_msgs::msg::PredictedObjects filtered_objects;
  filtered_objects.header = planner_data->predicted_objects.header;
  for (const auto & object : planner_data->predicted_objects.objects) {
    const auto is_pedestrian =
      std::find_if(object.classification.begin(), object.classification.end(), [](const auto & c) {
        return c.label == autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      }) != object.classification.end();
    if (is_pedestrian) continue;

    const auto is_coming_from_behind =
      motion_utils::calcSignedArcLength(
        ego_data.trajectory_points, ego_data.first_trajectory_idx,
        object.kinematics.initial_pose_with_covariance.pose.position) < 0.0;
    if (params.objects_ignore_behind_ego && is_coming_from_behind) {
      continue;
    }

    auto filtered_object = object;
    const auto is_invalid_predicted_path = [&](const auto & predicted_path) {
      const auto is_low_confidence = predicted_path.confidence < params.objects_min_confidence;
      const auto no_overlap_path = autoware::motion_utils::removeOverlapPoints(predicted_path.path);
      if (no_overlap_path.size() <= 1) return true;
      const auto lat_offset_to_current_ego = std::abs(
        autoware::motion_utils::calcLateralOffset(no_overlap_path, ego_data.pose.position));
      const auto is_crossing_ego =
        lat_offset_to_current_ego <=
        object.shape.dimensions.y / 2.0 + std::max(
                                            params.left_offset + params.extra_left_offset,
                                            params.right_offset + params.extra_right_offset);
      return is_low_confidence || is_crossing_ego;
    };
    if (params.objects_use_predicted_paths) {
      auto & predicted_paths = filtered_object.kinematics.predicted_paths;
      const auto new_end =
        std::remove_if(predicted_paths.begin(), predicted_paths.end(), is_invalid_predicted_path);
      predicted_paths.erase(new_end, predicted_paths.end());
      if (params.objects_cut_predicted_paths_beyond_red_lights)
        for (auto & predicted_path : predicted_paths)
          cut_predicted_path_beyond_red_lights(
            predicted_path, ego_data, filtered_object.shape.dimensions.x);
      predicted_paths.erase(
        std::remove_if(
          predicted_paths.begin(), predicted_paths.end(),
          [](const auto & p) { return p.path.empty(); }),
        predicted_paths.end());
    }

    if (!params.objects_use_predicted_paths || !filtered_object.kinematics.predicted_paths.empty())
      filtered_objects.objects.push_back(filtered_object);
  }
  return filtered_objects;
}

}  // namespace autoware::motion_velocity_planner::out_of_lane
