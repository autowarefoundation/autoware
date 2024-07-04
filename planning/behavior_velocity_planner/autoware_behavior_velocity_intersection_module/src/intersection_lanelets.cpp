// Copyright 2024 Tier IV, Inc.
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

#include "intersection_lanelets.hpp"

#include "util.hpp"

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <string>

namespace autoware::behavior_velocity_planner
{

void IntersectionLanelets::update(
  const bool is_prioritized, const InterpolatedPathInfo & interpolated_path_info,
  const autoware::universe_utils::LinearRing2d & footprint, const double vehicle_length,
  lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  is_prioritized_ = is_prioritized;
  // find the first conflicting/detection area polygon intersecting the path
  if (!first_conflicting_area_) {
    auto first = util::getFirstPointInsidePolygonsByFootprint(
      conflicting_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_conflicting_lane_ = conflicting_.at(first.value().second);
      first_conflicting_area_ = conflicting_area_.at(first.value().second);
    }
  }
  if (!first_attention_area_) {
    const auto first = util::getFirstPointInsidePolygonsByFootprint(
      attention_non_preceding_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_attention_lane_ = attention_non_preceding_.at(first.value().second);
      first_attention_area_ = attention_non_preceding_area_.at(first.value().second);
    }
  }
  if (first_attention_lane_ && !second_attention_lane_ && !second_attention_lane_empty_) {
    // remove first_attention_area_ and non-straight lanelets from attention_non_preceding
    lanelet::ConstLanelets attention_non_preceding_ex_first;
    lanelet::ConstLanelets sibling_first_attention_lanelets;
    for (const auto & previous : routing_graph_ptr->previous(first_attention_lane_.value())) {
      for (const auto & following : routing_graph_ptr->following(previous)) {
        sibling_first_attention_lanelets.push_back(following);
      }
    }
    for (const auto & ll : attention_non_preceding_) {
      // the sibling lanelets of first_attention_lanelet are ruled out
      if (lanelet::utils::contains(sibling_first_attention_lanelets, ll)) {
        continue;
      }
      if (std::string(ll.attributeOr("turn_direction", "else")).compare("straight") == 0) {
        attention_non_preceding_ex_first.push_back(ll);
      }
    }
    if (attention_non_preceding_ex_first.empty()) {
      second_attention_lane_empty_ = true;
    }
    const auto attention_non_preceding_ex_first_area =
      util::getPolygon3dFromLanelets(attention_non_preceding_ex_first);
    const auto second = util::getFirstPointInsidePolygonsByFootprint(
      attention_non_preceding_ex_first_area, interpolated_path_info, footprint, vehicle_length);
    if (second) {
      second_attention_lane_ = attention_non_preceding_ex_first.at(second.value().second);
      second_attention_area_ = second_attention_lane_.value().polygon3d();
    }
  }
}
}  // namespace autoware::behavior_velocity_planner
