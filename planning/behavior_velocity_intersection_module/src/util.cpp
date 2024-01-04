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

#include "util_type.hpp"

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace util
{

static std::optional<size_t> getDuplicatedPointIdx(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point)
{
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i).point.pose.position;

    constexpr double min_dist = 0.001;
    if (tier4_autoware_utils::calcDistance2d(p, point) < min_dist) {
      return i;
    }
  }

  return std::nullopt;
}

std::optional<size_t> insertPointIndex(
  const geometry_msgs::msg::Pose & in_pose,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  const auto duplicate_idx_opt = getDuplicatedPointIdx(*inout_path, in_pose.position);
  if (duplicate_idx_opt) {
    return duplicate_idx_opt.value();
  }

  const size_t closest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    inout_path->points, in_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  // vector.insert(i) inserts element on the left side of v[i]
  // the velocity need to be zero order hold(from prior point)
  int insert_idx = closest_idx;
  autoware_auto_planning_msgs::msg::PathPointWithLaneId inserted_point =
    inout_path->points.at(closest_idx);
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
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p,
  const std::set<lanelet::Id> & ids)
{
  for (const auto & pid : p.lane_ids) {
    if (ids.find(pid) != ids.end()) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<size_t, size_t>> findLaneIdsInterval(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & p, const std::set<lanelet::Id> & ids)
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
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));
  const auto area_2d = lanelet::utils::to2D(polygon).basicPolygon();
  for (auto i = start; i <= lane_end; ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = tier4_autoware_utils::transformVector(
      footprint, tier4_autoware_utils::pose2transform(base_pose));
    if (bg::intersects(path_footprint, area_2d)) {
      return std::make_optional<size_t>(i);
    }
  }
  return std::nullopt;
}

std::optional<size_t> getFirstPointInsidePolygon(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
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

std::pair<lanelet::ConstLanelets, std::vector<lanelet::ConstLanelets>>
mergeLaneletsByTopologicalSort(
  const lanelet::ConstLanelets & lanelets,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  const int n_node = lanelets.size();
  std::vector<std::vector<int>> adjacency(n_node);
  for (int dst = 0; dst < n_node; ++dst) {
    adjacency[dst].resize(n_node);
    for (int src = 0; src < n_node; ++src) {
      adjacency[dst][src] = false;
    }
  }
  std::set<lanelet::Id> lanelet_ids;
  std::unordered_map<lanelet::Id, int> id2ind;
  std::unordered_map<int, lanelet::Id> ind2id;
  std::unordered_map<lanelet::Id, lanelet::ConstLanelet> id2lanelet;
  int ind = 0;
  for (const auto & lanelet : lanelets) {
    lanelet_ids.insert(lanelet.id());
    const auto id = lanelet.id();
    id2ind[id] = ind;
    ind2id[ind] = id;
    id2lanelet[id] = lanelet;
    ind++;
  }
  // NOTE: this function aims to traverse the detection lanelet backward from ego side to farthest
  // side, so if lane B follows lane A on the routing_graph, adj[B][A] = true
  for (const auto & lanelet : lanelets) {
    const auto & followings = routing_graph_ptr->following(lanelet);
    const int dst = lanelet.id();
    for (const auto & following : followings) {
      if (const int src = following.id(); lanelet_ids.find(src) != lanelet_ids.end()) {
        adjacency[(id2ind[src])][(id2ind[dst])] = true;
      }
    }
  }
  // terminal node
  std::map<lanelet::Id, std::vector<lanelet::Id>> branches;
  auto has_no_previous = [&](const int node) {
    for (int dst = 0; dst < n_node; dst++) {
      if (adjacency[dst][node]) {
        return false;
      }
    }
    return true;
  };
  for (int src = 0; src < n_node; src++) {
    if (!has_no_previous(src)) {
      continue;
    }
    // So `src` has no previous lanelets
    branches[(ind2id[src])] = std::vector<lanelet::Id>{};
    auto & branch = branches[(ind2id[src])];
    lanelet::Id node_iter = ind2id[src];
    std::set<lanelet::Id> visited_ids;
    while (true) {
      const auto & destinations = adjacency[(id2ind[node_iter])];
      // NOTE: assuming detection lanelets have only one "previous"(on the routing_graph) lanelet
      const auto next = std::find(destinations.begin(), destinations.end(), true);
      if (next == destinations.end()) {
        branch.push_back(node_iter);
        break;
      }
      if (visited_ids.find(node_iter) != visited_ids.end()) {
        // loop detected
        break;
      }
      branch.push_back(node_iter);
      visited_ids.insert(node_iter);
      node_iter = ind2id[std::distance(destinations.begin(), next)];
    }
  }
  for (decltype(branches)::iterator it = branches.begin(); it != branches.end(); it++) {
    auto & branch = it->second;
    std::reverse(branch.begin(), branch.end());
  }
  lanelet::ConstLanelets merged;
  std::vector<lanelet::ConstLanelets> originals;
  for (const auto & [id, sub_ids] : branches) {
    if (sub_ids.size() == 0) {
      continue;
    }
    lanelet::ConstLanelets merge;
    originals.push_back(lanelet::ConstLanelets({}));
    auto & original = originals.back();
    for (const auto sub_id : sub_ids) {
      merge.push_back(id2lanelet[sub_id]);
      original.push_back(id2lanelet[sub_id]);
    }
    merged.push_back(lanelet::utils::combineLaneletsShape(merge));
  }
  return {merged, originals};
}

bool isOverTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const size_t target_idx)
{
  if (closest_idx == target_idx) {
    const geometry_msgs::msg::Pose target_pose = path.points.at(target_idx).point.pose;
    return planning_utils::isAheadOf(current_pose, target_pose);
  }
  return static_cast<bool>(closest_idx > target_idx);
}

bool isBeforeTargetIndex(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const geometry_msgs::msg::Pose & current_pose, const size_t target_idx)
{
  if (closest_idx == target_idx) {
    const geometry_msgs::msg::Pose target_pose = path.points.at(target_idx).point.pose;
    return planning_utils::isAheadOf(target_pose, current_pose);
  }
  return static_cast<bool>(target_idx > closest_idx);
}

std::optional<tier4_autoware_utils::Polygon2d> getIntersectionArea(
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

bool hasAssociatedTrafficLight(lanelet::ConstLanelet lane)
{
  std::optional<int> tl_id = std::nullopt;
  for (auto && tl_reg_elem : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    tl_id = tl_reg_elem->id();
    break;
  }
  return tl_id.has_value();
}

std::optional<InterpolatedPathInfo> generateInterpolatedPath(
  const lanelet::Id lane_id, const std::set<lanelet::Id> & associative_lane_ids,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path, const double ds,
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
  const autoware_auto_perception_msgs::msg::PredictedObjectKinematics & obj_state)
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

}  // namespace util
}  // namespace behavior_velocity_planner
