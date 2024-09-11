// Copyright 2020 Tier IV, Inc.
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

#include "util.hpp"

#include "interpolated_path_info.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_velocity_planner::util
{
namespace bg = boost::geometry;

static std::optional<size_t> getDuplicatedPointIdx(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const geometry_msgs::msg::Point & point)
{
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i).point.pose.position;

    constexpr double min_dist = 0.001;
    if (autoware::universe_utils::calcDistance2d(p, point) < min_dist) {
      return i;
    }
  }

  return std::nullopt;
}

std::optional<size_t> insertPointIndex(
  const geometry_msgs::msg::Pose & in_pose, tier4_planning_msgs::msg::PathWithLaneId * inout_path,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  const auto duplicate_idx_opt = getDuplicatedPointIdx(*inout_path, in_pose.position);
  if (duplicate_idx_opt) {
    return duplicate_idx_opt.value();
  }

  const size_t closest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    inout_path->points, in_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  // vector.insert(i) inserts element on the left side of v[i]
  // the velocity need to be zero order hold(from prior point)
  int insert_idx = closest_idx;
  tier4_planning_msgs::msg::PathPointWithLaneId inserted_point = inout_path->points.at(closest_idx);
  if (planning_utils::isAheadOf(in_pose, inout_path->points.at(closest_idx).point.pose)) {
    ++insert_idx;
  } else {
    // copy with velocity from prior point
    const size_t prior_ind = closest_idx > 0 ? closest_idx - 1 : 0;
    inserted_point.point.longitudinal_velocity_mps =
      inout_path->points.at(prior_ind).point.longitudinal_velocity_mps;
  }
  inserted_point.point.pose = in_pose;

  auto it = inout_path->points.begin() + insert_idx;
  inout_path->points.insert(it, inserted_point);

  return insert_idx;
}

bool hasLaneIds(
  const tier4_planning_msgs::msg::PathPointWithLaneId & p, const std::set<lanelet::Id> & ids)
{
  for (const auto & pid : p.lane_ids) {
    if (ids.find(pid) != ids.end()) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<size_t, size_t>> findLaneIdsInterval(
  const tier4_planning_msgs::msg::PathWithLaneId & p, const std::set<lanelet::Id> & ids)
{
  bool found = false;
  size_t start = 0;
  size_t end = p.points.size() > 0 ? p.points.size() - 1 : 0;
  if (start == end) {
    // there is only one point in the path
    return std::nullopt;
  }
  for (size_t i = 0; i < p.points.size(); ++i) {
    if (hasLaneIds(p.points.at(i), ids)) {
      if (!found) {
        // found interval for the first time
        found = true;
        start = i;
      }
    } else if (found) {
      // prior point was in the interval. interval ended
      end = i;
      break;
    }
  }
  start = start > 0 ? start - 1 : 0;  // the idx of last point before the interval
  return found ? std::make_optional(std::make_pair(start, end)) : std::nullopt;
}

std::optional<size_t> getFirstPointInsidePolygonByFootprint(
  const lanelet::CompoundPolygon3d & polygon, const InterpolatedPathInfo & interpolated_path_info,
  const autoware::universe_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));
  const auto area_2d = lanelet::utils::to2D(polygon).basicPolygon();
  for (auto i = start; i <= lane_end; ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = autoware::universe_utils::transformVector(
      footprint, autoware::universe_utils::pose2transform(base_pose));
    if (bg::intersects(path_footprint, area_2d)) {
      return std::make_optional<size_t>(i);
    }
  }
  return std::nullopt;
}

std::optional<std::pair<
  size_t /* the index of interpolated PathPoint*/, size_t /* the index of corresponding Polygon */>>
getFirstPointInsidePolygonsByFootprint(
  const std::vector<lanelet::CompoundPolygon3d> & polygons,
  const InterpolatedPathInfo & interpolated_path_info,
  const autoware::universe_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));

  for (size_t i = start; i <= lane_end; ++i) {
    const auto & pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = autoware::universe_utils::transformVector(
      footprint, autoware::universe_utils::pose2transform(pose));
    for (size_t j = 0; j < polygons.size(); ++j) {
      const auto area_2d = lanelet::utils::to2D(polygons.at(j)).basicPolygon();
      const bool is_in_polygon = bg::intersects(area_2d, path_footprint);
      if (is_in_polygon) {
        return std::make_optional<std::pair<size_t, size_t>>(i, j);
      }
    }
  }
  return std::nullopt;
}

std::optional<size_t> getFirstPointInsidePolygon(
  const tier4_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval, const lanelet::CompoundPolygon3d & polygon,
  const bool search_forward)
{
  // NOTE: if first point is already inside the polygon, returns nullopt
  const auto polygon_2d = lanelet::utils::to2D(polygon);
  if (search_forward) {
    const auto & p0 = path.points.at(lane_interval.first).point.pose.position;
    if (bg::within(to_bg2d(p0), polygon_2d)) {
      return std::nullopt;
    }
    for (size_t i = lane_interval.first; i <= lane_interval.second; ++i) {
      const auto & p = path.points.at(i).point.pose.position;
      const auto is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        return std::make_optional<size_t>(i);
      }
    }
  } else {
    const auto & p0 = path.points.at(lane_interval.second).point.pose.position;
    if (bg::within(to_bg2d(p0), polygon_2d)) {
      return std::nullopt;
    }
    for (size_t i = lane_interval.second; i >= lane_interval.first; --i) {
      const auto & p = path.points.at(i).point.pose.position;
      const auto is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        return std::make_optional<size_t>(i);
      }
      if (i == 0) {
        break;
      }
    }
  }
  return std::nullopt;
}

void retrievePathsBackward(
  const std::vector<std::vector<bool>> & adjacency, const size_t src_ind,
  const std::vector<size_t> & visited_inds, std::vector<std::vector<size_t>> & paths)
{
  const auto & nexts = adjacency.at(src_ind);
  const bool is_terminal = (std::find(nexts.begin(), nexts.end(), true) == nexts.end());
  if (is_terminal) {
    std::vector<size_t> path(visited_inds.begin(), visited_inds.end());
    path.push_back(src_ind);
    paths.emplace_back(std::move(path));
    return;
  }
  for (size_t next = 0; next < nexts.size(); next++) {
    if (!nexts.at(next)) {
      continue;
    }
    if (std::find(visited_inds.begin(), visited_inds.end(), next) != visited_inds.end()) {
      // loop detected
      std::vector<size_t> path(visited_inds.begin(), visited_inds.end());
      path.push_back(src_ind);
      paths.emplace_back(std::move(path));
      continue;
    }
    auto new_visited_inds = visited_inds;
    new_visited_inds.push_back(src_ind);
    retrievePathsBackward(adjacency, next, new_visited_inds, paths);
  }
  return;
}

std::pair<lanelet::ConstLanelets, std::vector<lanelet::ConstLanelets>>
mergeLaneletsByTopologicalSort(
  const lanelet::ConstLanelets & lanelets, const lanelet::ConstLanelets & terminal_lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  std::set<lanelet::Id> lanelet_Ids;
  std::unordered_map<lanelet::Id, size_t> Id2ind;
  std::unordered_map<size_t, lanelet::Id> ind2Id;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelet> Id2lanelet;
  for (const auto & lanelet : lanelets) {
    size_t ind = ind2Id.size();
    const auto Id = lanelet.id();
    lanelet_Ids.insert(Id);
    Id2ind[Id] = ind;
    ind2Id[ind] = Id;
    Id2lanelet[Id] = lanelet;
  }
  std::set<size_t> terminal_inds;
  for (const auto & terminal_lanelet : terminal_lanelets) {
    if (Id2ind.count(terminal_lanelet.id()) > 0) {
      terminal_inds.insert(Id2ind[terminal_lanelet.id()]);
    }
  }

  // create adjacency matrix
  const auto n_node = lanelets.size();
  std::vector<std::vector<bool>> adjacency(n_node);
  for (size_t dst = 0; dst < n_node; ++dst) {
    adjacency[dst].resize(n_node);
    for (size_t src = 0; src < n_node; ++src) {
      adjacency[dst][src] = false;
    }
  }
  // NOTE: this function aims to traverse the detection lanelet in the lane direction, so if lane B
  // follows lane A on the routing_graph, adj[A][B] = true
  for (const auto & lanelet : lanelets) {
    const auto & followings = routing_graph_ptr->following(lanelet);
    const auto src = lanelet.id();
    for (const auto & following : followings) {
      if (const auto dst = following.id(); lanelet_Ids.find(dst) != lanelet_Ids.end()) {
        adjacency[(Id2ind[dst])][(Id2ind[src])] = true;
      }
    }
  }

  std::unordered_map<size_t, std::vector<std::vector<size_t>>> branches;
  for (const auto & terminal_ind : terminal_inds) {
    std::vector<std::vector<size_t>> paths;
    std::vector<size_t> visited;
    retrievePathsBackward(adjacency, terminal_ind, visited, paths);
    branches[terminal_ind] = std::move(paths);
  }

  for (auto it = branches.begin(); it != branches.end(); it++) {
    auto & paths = it->second;
    for (auto & path : paths) {
      std::reverse(path.begin(), path.end());
    }
  }
  lanelet::ConstLanelets merged;
  std::vector<lanelet::ConstLanelets> originals;
  for (const auto & [ind, sub_branches] : branches) {
    if (sub_branches.size() == 0) {
      continue;
    }
    for (const auto & sub_inds : sub_branches) {
      lanelet::ConstLanelets to_be_merged;
      originals.push_back(lanelet::ConstLanelets({}));
      auto & original = originals.back();
      for (const auto & sub_ind : sub_inds) {
        to_be_merged.push_back(Id2lanelet[ind2Id[sub_ind]]);
        original.push_back(Id2lanelet[ind2Id[sub_ind]]);
      }
      merged.push_back(lanelet::utils::combineLaneletsShape(to_be_merged));
    }
  }
  return {merged, originals};
}

bool isOverTargetIndex(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const size_t target_idx)
{
  if (closest_idx == target_idx) {
    const geometry_msgs::msg::Pose target_pose = path.points.at(target_idx).point.pose;
    return planning_utils::isAheadOf(current_pose, target_pose);
  }
  return static_cast<bool>(closest_idx > target_idx);
}

std::optional<autoware::universe_utils::Polygon2d> getIntersectionArea(
  lanelet::ConstLanelet assigned_lane, lanelet::LaneletMapConstPtr lanelet_map_ptr)
{
  const std::string area_id_str = assigned_lane.attributeOr("intersection_area", "else");
  if (area_id_str == "else") return std::nullopt;

  const lanelet::Id area_id = std::atoi(area_id_str.c_str());
  const auto poly_3d = lanelet_map_ptr->polygonLayer.get(area_id);
  Polygon2d poly{};
  for (const auto & p : poly_3d) poly.outer().emplace_back(p.x(), p.y());
  return std::make_optional(poly);
}

std::optional<InterpolatedPathInfo> generateInterpolatedPath(
  const lanelet::Id lane_id, const std::set<lanelet::Id> & associative_lane_ids,
  const tier4_planning_msgs::msg::PathWithLaneId & input_path, const double ds,
  const rclcpp::Logger logger)
{
  InterpolatedPathInfo interpolated_path_info;
  if (!splineInterpolate(input_path, ds, interpolated_path_info.path, logger)) {
    return std::nullopt;
  }
  interpolated_path_info.ds = ds;
  interpolated_path_info.lane_id = lane_id;
  interpolated_path_info.associative_lane_ids = associative_lane_ids;
  interpolated_path_info.lane_id_interval =
    findLaneIdsInterval(interpolated_path_info.path, associative_lane_ids);
  return interpolated_path_info;
}

geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_perception_msgs::msg::PredictedObjectKinematics & obj_state)
{
  if (obj_state.initial_twist_with_covariance.twist.linear.x >= 0) {
    return obj_state.initial_pose_with_covariance.pose;
  }

  // When the object velocity is negative, invert orientation (yaw)
  auto obj_pose = obj_state.initial_pose_with_covariance.pose;
  double yaw, pitch, roll;
  tf2::getEulerYPR(obj_pose.orientation, yaw, pitch, roll);
  tf2::Quaternion inv_q;
  inv_q.setRPY(roll, pitch, yaw + M_PI);
  obj_pose.orientation = tf2::toMsg(inv_q);
  return obj_pose;
}

std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & ll_vec)
{
  std::vector<lanelet::CompoundPolygon3d> polys;
  for (auto && ll : ll_vec) {
    polys.push_back(ll.polygon3d());
  }
  return polys;
}

}  // namespace autoware::behavior_velocity_planner::util
