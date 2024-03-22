// Copyright 2023-2024 TIER IV, Inc.
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

#include <motion_utils/trajectory/trajectory.hpp>
#include <traffic_light_utils/traffic_light_utils.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>

namespace behavior_velocity_planner::out_of_lane
{
void cut_predicted_path_beyond_line(
  autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const lanelet::BasicLineString2d & stop_line, const double object_front_overhang)
{
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
      arc_length += tier4_autoware_utils::calcDistance2d(
        predicted_path.path[cut_idx], predicted_path.path[cut_idx - 1]);
      --cut_idx;
    }
    predicted_path.path.resize(cut_idx);
  }
}

std::optional<const lanelet::BasicLineString2d> find_next_stop_line(
  const autoware_auto_perception_msgs::msg::PredictedPath & path, const PlannerData & planner_data)
{
  lanelet::ConstLanelets lanelets;
  lanelet::BasicLineString2d query_line;
  for (const auto & p : path.path) query_line.emplace_back(p.position.x, p.position.y);
  const auto query_results = lanelet::geometry::findWithin2d(
    planner_data.route_handler_->getLaneletMapPtr()->laneletLayer, query_line);
  for (const auto & r : query_results) lanelets.push_back(r.second);
  for (const auto & ll : lanelets) {
    for (const auto & element : ll.regulatoryElementsAs<lanelet::TrafficLight>()) {
      const auto traffic_signal_stamped = planner_data.getTrafficSignal(element->id());
      if (
        traffic_signal_stamped.has_value() && element->stopLine().has_value() &&
        traffic_light_utils::isTrafficSignalStop(ll, traffic_signal_stamped.value().signal)) {
        lanelet::BasicLineString2d stop_line;
        for (const auto & p : *(element->stopLine())) stop_line.emplace_back(p.x(), p.y());
        return stop_line;
      }
    }
  }
  return std::nullopt;
}

void cut_predicted_path_beyond_red_lights(
  autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const PlannerData & planner_data, const double object_front_overhang)
{
  const auto stop_line = find_next_stop_line(predicted_path, planner_data);
  if (stop_line) {
    // we use a longer stop line to also cut predicted paths that slightly go around the stop line
    auto longer_stop_line = *stop_line;
    const auto diff = stop_line->back() - stop_line->front();
    longer_stop_line.front() -= diff * 0.5;
    longer_stop_line.back() += diff * 0.5;
    cut_predicted_path_beyond_line(predicted_path, longer_stop_line, object_front_overhang);
  }
}

autoware_auto_perception_msgs::msg::PredictedObjects filter_predicted_objects(
  const PlannerData & planner_data, const EgoData & ego_data, const PlannerParam & params)
{
  autoware_auto_perception_msgs::msg::PredictedObjects filtered_objects;
  filtered_objects.header = planner_data.predicted_objects->header;
  for (const auto & object : planner_data.predicted_objects->objects) {
    const auto is_pedestrian =
      std::find_if(object.classification.begin(), object.classification.end(), [](const auto & c) {
        return c.label == autoware_auto_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
      }) != object.classification.end();
    if (is_pedestrian) continue;

    auto filtered_object = object;
    const auto is_invalid_predicted_path = [&](const auto & predicted_path) {
      const auto is_low_confidence = predicted_path.confidence < params.objects_min_confidence;
      const auto no_overlap_path = motion_utils::removeOverlapPoints(predicted_path.path);
      if (no_overlap_path.size() <= 1) return true;
      const auto lat_offset_to_current_ego =
        std::abs(motion_utils::calcLateralOffset(no_overlap_path, ego_data.pose.position));
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
            predicted_path, planner_data, filtered_object.shape.dimensions.x);
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

}  // namespace behavior_velocity_planner::out_of_lane
