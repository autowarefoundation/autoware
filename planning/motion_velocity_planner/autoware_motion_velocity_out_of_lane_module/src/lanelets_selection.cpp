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

#include "lanelets_selection.hpp"

#include "types.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/union.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>

namespace autoware::motion_velocity_planner::out_of_lane
{

namespace
{
bool is_road_lanelet(const lanelet::ConstLanelet & lanelet)
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
}
}  // namespace

lanelet::ConstLanelets consecutive_lanelets(
  const route_handler::RouteHandler & route_handler, const lanelet::ConstLanelet & lanelet)
{
  lanelet::ConstLanelets consecutives = route_handler.getRoutingGraphPtr()->following(lanelet);
  const auto previous = route_handler.getRoutingGraphPtr()->previous(lanelet);
  consecutives.insert(consecutives.end(), previous.begin(), previous.end());
  return consecutives;
}

lanelet::ConstLanelets get_missing_lane_change_lanelets(
  const lanelet::ConstLanelets & trajectory_lanelets,
  const route_handler::RouteHandler & route_handler)
{
  lanelet::ConstLanelets missing_lane_change_lanelets;
  const auto & routing_graph = *route_handler.getRoutingGraphPtr();
  lanelet::ConstLanelets adjacents;
  lanelet::ConstLanelets consecutives;
  for (const auto & ll : trajectory_lanelets) {
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
             !contains_lanelet(trajectory_lanelets, l.id()) &&
             contains_lanelet(consecutives, l.id());
    });
  return missing_lane_change_lanelets;
}

lanelet::ConstLanelets calculate_trajectory_lanelets(
  const EgoData & ego_data, const route_handler::RouteHandler & route_handler)
{
  const auto lanelet_map_ptr = route_handler.getLaneletMapPtr();
  lanelet::ConstLanelets trajectory_lanelets;
  lanelet::BasicLineString2d trajectory_ls;
  for (const auto & p : ego_data.trajectory_points)
    trajectory_ls.emplace_back(p.pose.position.x, p.pose.position.y);
  const auto candidates =
    lanelet_map_ptr->laneletLayer.search(lanelet::geometry::boundingBox2d(trajectory_ls));
  for (const auto & ll : candidates) {
    if (
      is_road_lanelet(ll) &&
      !boost::geometry::disjoint(trajectory_ls, ll.polygon2d().basicPolygon())) {
      trajectory_lanelets.push_back(ll);
    }
  }
  const auto missing_lanelets =
    get_missing_lane_change_lanelets(trajectory_lanelets, route_handler);
  trajectory_lanelets.insert(
    trajectory_lanelets.end(), missing_lanelets.begin(), missing_lanelets.end());
  return trajectory_lanelets;
}

lanelet::ConstLanelets calculate_ignored_lanelets(
  const lanelet::ConstLanelets & trajectory_lanelets,
  const route_handler::RouteHandler & route_handler)
{
  lanelet::ConstLanelets ignored_lanelets;
  // ignore lanelets directly preceding a trajectory lanelet
  for (const auto & trajectory_lanelet : trajectory_lanelets) {
    for (const auto & ll : route_handler.getPreviousLanelets(trajectory_lanelet)) {
      const auto is_trajectory_lanelet = contains_lanelet(trajectory_lanelets, ll.id());
      if (!is_trajectory_lanelet) ignored_lanelets.push_back(ll);
    }
  }
  return ignored_lanelets;
}

void calculate_drivable_lane_polygons(
  EgoData & ego_data, const route_handler::RouteHandler & route_handler)
{
  const auto route_lanelets = calculate_trajectory_lanelets(ego_data, route_handler);
  const auto ignored_lanelets =
    out_of_lane::calculate_ignored_lanelets(route_lanelets, route_handler);
  for (const auto & ll : route_lanelets) {
    out_of_lane::Polygons tmp;
    boost::geometry::union_(ego_data.drivable_lane_polygons, ll.polygon2d().basicPolygon(), tmp);
    ego_data.drivable_lane_polygons = tmp;
  }
  for (const auto & ll : ignored_lanelets) {
    out_of_lane::Polygons tmp;
    boost::geometry::union_(ego_data.drivable_lane_polygons, ll.polygon2d().basicPolygon(), tmp);
    ego_data.drivable_lane_polygons = tmp;
  }
}

void calculate_overlapped_lanelets(
  OutOfLanePoint & out_of_lane_point, const route_handler::RouteHandler & route_handler)
{
  out_of_lane_point.overlapped_lanelets = lanelet::ConstLanelets();
  const auto candidates = route_handler.getLaneletMapPtr()->laneletLayer.search(
    lanelet::geometry::boundingBox2d(out_of_lane_point.outside_ring));
  for (const auto & ll : candidates) {
    if (
      is_road_lanelet(ll) && !contains_lanelet(out_of_lane_point.overlapped_lanelets, ll.id()) &&
      boost::geometry::within(out_of_lane_point.outside_ring, ll.polygon2d().basicPolygon())) {
      out_of_lane_point.overlapped_lanelets.push_back(ll);
    }
  }
}

void calculate_overlapped_lanelets(
  OutOfLaneData & out_of_lane_data, const route_handler::RouteHandler & route_handler)
{
  for (auto it = out_of_lane_data.outside_points.begin();
       it != out_of_lane_data.outside_points.end();) {
    calculate_overlapped_lanelets(*it, route_handler);
    if (it->overlapped_lanelets.empty()) {
      // do not keep out of lane points that do not overlap any lanelet
      out_of_lane_data.outside_points.erase(it);
    } else {
      ++it;
    }
  }
}
}  // namespace autoware::motion_velocity_planner::out_of_lane
