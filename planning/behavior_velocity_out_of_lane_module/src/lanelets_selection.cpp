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

lanelet::ConstLanelets consecutive_lanelets(
  const route_handler::RouteHandler & route_handler, const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets consecutives = route_handler.getRoutingGraphPtr()->following(lanelet);
  const auto previous = route_handler.getRoutingGraphPtr()->previous(lanelet);
  consecutives.insert(consecutives.end(), previous.begin(), previous.end());
  return consecutives;
}

lanelet::ConstLanelets get_missing_lane_change_lanelets(
  lanelet::ConstLanelets & path_lanelets, const route_handler::RouteHandler & route_handler)
{
  lanelet::ConstLanelets missing_lane_change_lanelets;
  const auto & routing_graph = *route_handler.getRoutingGraphPtr();
  lanelet::ConstLanelets adjacents;
  lanelet::ConstLanelets consecutives;
  for (const auto & ll : path_lanelets) {
    const auto consecutives_of_ll = consecutive_lanelets(route_handler, ll);
    std::copy_if(
      consecutives_of_ll.begin(), consecutives_of_ll.end(), std::back_inserter(consecutives),
      [&](const auto & l) { return !contains_lanelet(consecutives, l.id()); });
    const auto adjacents_of_ll = routing_graph.besides(ll);
    std::copy_if(
      adjacents_of_ll.begin(), adjacents_of_ll.end(), std::back_inserter(adjacents),
      [&](const auto & l) { return !contains_lanelet(adjacents, l.id()); });
  }
  std::copy_if(
    adjacents.begin(), adjacents.end(), std::back_inserter(missing_lane_change_lanelets),
    [&](const auto & l) {
      return !contains_lanelet(missing_lane_change_lanelets, l.id()) &&
             !contains_lanelet(path_lanelets, l.id()) && contains_lanelet(consecutives, l.id());
    });
  return missing_lane_change_lanelets;
}

lanelet::ConstLanelets calculate_path_lanelets(
  const EgoData & ego_data, const route_handler::RouteHandler & route_handler)
{
  const auto lanelet_map_ptr = route_handler.getLaneletMapPtr();
  lanelet::ConstLanelets path_lanelets;
  lanelet::BasicLineString2d path_ls;
  for (const auto & p : ego_data.path.points)
    path_ls.emplace_back(p.point.pose.position.x, p.point.pose.position.y);
  for (const auto & dist_lanelet :
       lanelet::geometry::findWithin2d(lanelet_map_ptr->laneletLayer, path_ls)) {
    if (!contains_lanelet(path_lanelets, dist_lanelet.second.id()))
      path_lanelets.push_back(dist_lanelet.second);
  }
  const auto missing_lanelets = get_missing_lane_change_lanelets(path_lanelets, route_handler);
  path_lanelets.insert(path_lanelets.end(), missing_lanelets.begin(), missing_lanelets.end());
  return path_lanelets;
}

lanelet::ConstLanelets calculate_ignored_lanelets(
  const EgoData & ego_data, const lanelet::ConstLanelets & path_lanelets,
  const route_handler::RouteHandler & route_handler, const PlannerParam & params)
{
  lanelet::ConstLanelets ignored_lanelets;
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
    std::max(params.slow_dist_threshold, params.stop_dist_threshold) + params.front_offset +
      params.extra_front_offset);
  for (const auto & ll : lanelets_within_range) {
    if (std::string(ll.second.attributeOr(lanelet::AttributeName::Subtype, "none")) != "road")
      continue;
    const auto is_path_lanelet = contains_lanelet(path_lanelets, ll.second.id());
    const auto is_ignored_lanelet = contains_lanelet(ignored_lanelets, ll.second.id());
    if (!is_path_lanelet && !is_ignored_lanelet) other_lanelets.push_back(ll.second);
  }
  return other_lanelets;
}
}  // namespace behavior_velocity_planner::out_of_lane
