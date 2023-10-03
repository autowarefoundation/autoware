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

#include "lanelets_selection.hpp"

#include <behavior_velocity_planner_common/utilization/util.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <string>

namespace behavior_velocity_planner::out_of_lane
{
lanelet::ConstLanelets calculate_ignored_lanelets(
  const EgoData & ego_data, const lanelet::ConstLanelets & path_lanelets,
  const route_handler::RouteHandler & route_handler, const PlannerParam & params)
{
  lanelet::ConstLanelets ignored_lanelets;
  // ignore lanelets that follows path lanelets
  for (const auto & path_lanelet : path_lanelets)
    for (const auto & following : route_handler.getRoutingGraphPtr()->following(path_lanelet))
      if (!contains_lanelet(path_lanelets, following.id())) ignored_lanelets.push_back(following);
  // ignore lanelets directly behind ego
  const auto behind =
    planning_utils::calculateOffsetPoint2d(ego_data.pose, params.rear_offset, 0.0);
  const lanelet::BasicPoint2d behind_point(behind.x(), behind.y());
  const auto behind_lanelets = lanelet::geometry::findWithin2d(
    route_handler.getLaneletMapPtr()->laneletLayer, behind_point, 0.0);
  for (const auto & l : behind_lanelets) {
    const auto is_path_lanelet = contains_lanelet(path_lanelets, l.second.id());
    if (!is_path_lanelet) ignored_lanelets.push_back(l.second);
  }
  return ignored_lanelets;
}

lanelet::ConstLanelets calculate_other_lanelets(
  const EgoData & ego_data, const lanelet::ConstLanelets & path_lanelets,
  const lanelet::ConstLanelets & ignored_lanelets,
  const route_handler::RouteHandler & route_handler, const PlannerParam & params)
{
  lanelet::ConstLanelets other_lanelets;
  const lanelet::BasicPoint2d ego_point(ego_data.pose.position.x, ego_data.pose.position.y);
  const auto lanelets_within_range = lanelet::geometry::findWithin2d(
    route_handler.getLaneletMapPtr()->laneletLayer, ego_point,
    std::max(params.slow_dist_threshold, params.stop_dist_threshold));
  const auto is_overlapped_by_path_lanelets = [&](const auto & l) {
    for (const auto & path_ll : path_lanelets) {
      if (
        boost::geometry::overlaps(
          path_ll.polygon2d().basicPolygon(), l.polygon2d().basicPolygon()) ||
        boost::geometry::within(path_ll.polygon2d().basicPolygon(), l.polygon2d().basicPolygon()) ||
        boost::geometry::within(l.polygon2d().basicPolygon(), path_ll.polygon2d().basicPolygon())) {
        return true;
      }
    }
    return false;
  };
  for (const auto & ll : lanelets_within_range) {
    if (std::string(ll.second.attributeOr(lanelet::AttributeName::Subtype, "none")) != "road")
      continue;
    const auto is_path_lanelet = contains_lanelet(path_lanelets, ll.second.id());
    const auto is_ignored_lanelet = contains_lanelet(ignored_lanelets, ll.second.id());
    if (!is_path_lanelet && !is_ignored_lanelet && !is_overlapped_by_path_lanelets(ll.second))
      other_lanelets.push_back(ll.second);
  }
  return other_lanelets;
}
}  // namespace behavior_velocity_planner::out_of_lane
