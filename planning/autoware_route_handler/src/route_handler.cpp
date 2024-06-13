// Copyright 2021-2024 TIER IV, Inc.
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

#include "autoware_route_handler/route_handler.hpp"

#include <autoware_utils/math/normalization.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/route_checker.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::route_handler
{
namespace
{
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::Path;
using geometry_msgs::msg::Pose;
using lanelet::utils::to2D;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

bool exists(const std::vector<LaneletPrimitive> & primitives, const int64_t & id)
{
  for (const auto & p : primitives) {
    if (p.id == id) {
      return true;
    }
  }
  return false;
}

template <typename T>
bool exists(const std::vector<T> & vectors, const T & item)
{
  for (const auto & i : vectors) {
    if (i == item) {
      return true;
    }
  }
  return false;
}

lanelet::ConstPoint3d get3DPointFrom2DArcLength(
  const lanelet::ConstLanelets & lanelet_sequence, const double s)
{
  double accumulated_distance2d = 0;
  for (const auto & llt : lanelet_sequence) {
    const auto & centerline = llt.centerline();
    lanelet::ConstPoint3d prev_pt;
    if (!centerline.empty()) {
      prev_pt = centerline.front();
    }
    for (const auto & pt : centerline) {
      const double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated_distance2d + distance2d > s) {
        const double ratio = (s - accumulated_distance2d) / distance2d;
        const auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d{
          lanelet::InvalId, interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z()};
      }
      accumulated_distance2d += distance2d;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d{};
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered_path;
  for (const auto & pt : input_path.points) {
    if (filtered_path.points.empty()) {
      filtered_path.points.push_back(pt);
      continue;
    }

    constexpr double min_dist = 0.001;
    if (
      tier4_autoware_utils::calcDistance3d(filtered_path.points.back().point, pt.point) <
      min_dist) {
      filtered_path.points.back().lane_ids.push_back(pt.lane_ids.front());
      filtered_path.points.back().point.longitudinal_velocity_mps = std::min(
        pt.point.longitudinal_velocity_mps,
        filtered_path.points.back().point.longitudinal_velocity_mps);
    } else {
      filtered_path.points.push_back(pt);
    }
  }
  filtered_path.left_bound = input_path.left_bound;
  filtered_path.right_bound = input_path.right_bound;
  return filtered_path;
}

std::string toString(const geometry_msgs::msg::Pose & pose)
{
  std::stringstream ss;
  ss << "(" << pose.position.x << ", " << pose.position.y << "," << pose.position.z << ")";
  return ss.str();
}

bool isClose(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, const double epsilon)
{
  return std::abs(p1.x - p2.x) < epsilon && std::abs(p1.y - p2.y) < epsilon;
}

PiecewiseReferencePoints convertWaypointsToReferencePoints(
  const std::vector<geometry_msgs::msg::Point> & piecewise_waypoints)
{
  PiecewiseReferencePoints piecewise_ref_points;
  for (const auto & piecewise_waypoint : piecewise_waypoints) {
    piecewise_ref_points.push_back(ReferencePoint{true, piecewise_waypoint});
  }
  return piecewise_ref_points;
}

template <typename T>
bool isIndexWithinVector(const std::vector<T> & vec, const int index)
{
  return 0 <= index && index < static_cast<int>(vec.size());
}

template <typename T>
void removeIndicesFromVector(std::vector<T> & vec, std::vector<size_t> indices)
{
  // sort indices in a descending order
  std::sort(indices.begin(), indices.end(), std::greater<int>());

  // remove indices from vector
  for (const size_t index : indices) {
    vec.erase(vec.begin() + index);
  }
}

lanelet::ArcCoordinates calcArcCoordinates(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & point)
{
  return lanelet::geometry::toArcCoordinates(
    to2D(lanelet.centerline()),
    to2D(lanelet::utils::conversion::toLaneletPoint(point)).basicPoint());
}
}  // namespace

RouteHandler::RouteHandler(const LaneletMapBin & map_msg)
{
  setMap(map_msg);
  route_ptr_ = nullptr;
}

void RouteHandler::setMap(const LaneletMapBin & map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  const lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  const lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  const lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);

  is_map_msg_ready_ = true;
  is_handler_ready_ = false;

  setLaneletsFromRouteMsg();
}

bool RouteHandler::isRouteLooped(const RouteSections & route_sections)
{
  std::set<lanelet::Id> lane_primitives;
  for (const auto & route_section : route_sections) {
    for (const auto & primitive : route_section.primitives) {
      if (lane_primitives.find(primitive.id) != lane_primitives.end()) {
        return true;  // find duplicated id
      }
      lane_primitives.emplace(primitive.id);
    }
  }
  return false;
}

void RouteHandler::setRoute(const LaneletRoute & route_msg)
{
  if (!isRouteLooped(route_msg.segments)) {
    // if get not modified route but new route, reset original start pose
    if (!route_ptr_ || route_ptr_->uuid != route_msg.uuid) {
      original_start_pose_ = route_msg.start_pose;
      original_goal_pose_ = route_msg.goal_pose;
    }
    route_ptr_ = std::make_shared<LaneletRoute>(route_msg);
    is_handler_ready_ = false;
    setLaneletsFromRouteMsg();
  } else {
    RCLCPP_ERROR(
      logger_,
      "Loop detected within route! Currently, no loop is allowed for route! Using previous route");
  }
}

bool RouteHandler::isHandlerReady() const
{
  return is_handler_ready_;
}

void RouteHandler::setRouteLanelets(const lanelet::ConstLanelets & path_lanelets)
{
  if (!path_lanelets.empty()) {
    const auto & first_lanelet = path_lanelets.front();
    start_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, first_lanelet);
    const auto & last_lanelet = path_lanelets.back();
    goal_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, last_lanelet);
  }

  // set route lanelets
  std::unordered_set<lanelet::Id> route_lanelets_id;
  std::unordered_set<lanelet::Id> candidate_lanes_id;
  for (const auto & lane : path_lanelets) {
    route_lanelets_id.insert(lane.id());
    const auto right_relations = routing_graph_ptr_->rightRelations(lane);
    for (const auto & right_relation : right_relations) {
      if (right_relation.relationType == lanelet::routing::RelationType::Right) {
        route_lanelets_id.insert(right_relation.lanelet.id());
      } else if (right_relation.relationType == lanelet::routing::RelationType::AdjacentRight) {
        candidate_lanes_id.insert(right_relation.lanelet.id());
      }
    }
    const auto left_relations = routing_graph_ptr_->leftRelations(lane);
    for (const auto & left_relation : left_relations) {
      if (left_relation.relationType == lanelet::routing::RelationType::Left) {
        route_lanelets_id.insert(left_relation.lanelet.id());
      } else if (left_relation.relationType == lanelet::routing::RelationType::AdjacentLeft) {
        candidate_lanes_id.insert(left_relation.lanelet.id());
      }
    }
  }

  //  check if candidates are really part of route
  for (const auto & candidate_id : candidate_lanes_id) {
    lanelet::ConstLanelet lanelet = lanelet_map_ptr_->laneletLayer.get(candidate_id);
    auto previous_lanelets = routing_graph_ptr_->previous(lanelet);
    bool is_connected_to_main_lanes_prev = false;
    bool is_connected_to_candidate_prev = true;
    if (exists(start_lanelets_, lanelet)) {
      is_connected_to_candidate_prev = false;
    }
    while (!previous_lanelets.empty() && is_connected_to_candidate_prev &&
           !is_connected_to_main_lanes_prev) {
      is_connected_to_candidate_prev = false;

      for (const auto & prev_lanelet : previous_lanelets) {
        if (route_lanelets_id.find(prev_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_prev = true;
          break;
        }
        if (exists(start_lanelets_, prev_lanelet)) {
          break;
        }

        if (candidate_lanes_id.find(prev_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_prev = true;
          previous_lanelets = routing_graph_ptr_->previous(prev_lanelet);
          break;
        }
      }
    }

    auto following_lanelets = routing_graph_ptr_->following(lanelet);
    bool is_connected_to_main_lanes_next = false;
    bool is_connected_to_candidate_next = true;
    if (exists(goal_lanelets_, lanelet)) {
      is_connected_to_candidate_next = false;
    }
    while (!following_lanelets.empty() && is_connected_to_candidate_next &&
           !is_connected_to_main_lanes_next) {
      is_connected_to_candidate_next = false;
      for (const auto & next_lanelet : following_lanelets) {
        if (route_lanelets_id.find(next_lanelet.id()) != route_lanelets_id.end()) {
          is_connected_to_main_lanes_next = true;
          break;
        }
        if (exists(goal_lanelets_, next_lanelet)) {
          break;
        }
        if (candidate_lanes_id.find(next_lanelet.id()) != candidate_lanes_id.end()) {
          is_connected_to_candidate_next = true;
          following_lanelets = routing_graph_ptr_->following(next_lanelet);
          break;
        }
      }
    }

    if (is_connected_to_main_lanes_next && is_connected_to_main_lanes_prev) {
      route_lanelets_id.insert(candidate_id);
    }
  }

  route_lanelets_.clear();
  route_lanelets_.reserve(route_lanelets_id.size());
  for (const auto & id : route_lanelets_id) {
    route_lanelets_.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  is_handler_ready_ = true;
}

void RouteHandler::clearRoute()
{
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  start_lanelets_.clear();
  goal_lanelets_.clear();
  route_ptr_ = nullptr;
  is_handler_ready_ = false;
}

void RouteHandler::setLaneletsFromRouteMsg()
{
  if (!route_ptr_ || !is_map_msg_ready_) {
    return;
  }
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  const bool is_route_valid = lanelet::utils::route::isRouteValid(*route_ptr_, lanelet_map_ptr_);
  if (!is_route_valid) {
    return;
  }

  size_t primitive_size{0};
  for (const auto & route_section : route_ptr_->segments) {
    primitive_size += route_section.primitives.size();
  }
  route_lanelets_.reserve(primitive_size);

  for (const auto & route_section : route_ptr_->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      route_lanelets_.push_back(llt);
      if (id == route_section.preferred_primitive.id) {
        preferred_lanelets_.push_back(llt);
      }
    }
  }
  goal_lanelets_.clear();
  start_lanelets_.clear();
  if (!route_ptr_->segments.empty()) {
    goal_lanelets_.reserve(route_ptr_->segments.back().primitives.size());
    for (const auto & primitive : route_ptr_->segments.back().primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      goal_lanelets_.push_back(llt);
    }
    start_lanelets_.reserve(route_ptr_->segments.front().primitives.size());
    for (const auto & primitive : route_ptr_->segments.front().primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      start_lanelets_.push_back(llt);
    }
  }
  is_handler_ready_ = true;
}

lanelet::ConstPolygon3d RouteHandler::getIntersectionAreaById(const lanelet::Id id) const
{
  return lanelet_map_ptr_->polygonLayer.get(id);
}

Header RouteHandler::getRouteHeader() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getRouteHeader: Route has not been set yet");
    return Header();
  }
  return route_ptr_->header;
}

UUID RouteHandler::getRouteUuid() const
{
  if (!route_ptr_) {
    RCLCPP_WARN_SKIPFIRST(logger_, "[Route Handler] getRouteUuid: Route has not been set yet");
    return UUID();
  }
  return route_ptr_->uuid;
}

bool RouteHandler::isAllowedGoalModification() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getRouteUuid: Route has not been set yet");
    return false;
  }
  return route_ptr_->allow_modification;
}

std::vector<lanelet::ConstLanelet> RouteHandler::getLanesBeforePose(
  const geometry_msgs::msg::Pose & pose, const double length) const
{
  lanelet::ConstLanelet pose_lanelet;
  if (!getClosestLaneletWithinRoute(pose, &pose_lanelet)) {
    return std::vector<lanelet::ConstLanelet>{};
  }

  const double min_preceding_length = length;
  const auto preceding_lanes_vec = lanelet::utils::query::getPrecedingLaneletSequences(
    routing_graph_ptr_, pose_lanelet, min_preceding_length);
  if (preceding_lanes_vec.empty()) {
    return std::vector<lanelet::ConstLanelet>{};
  }

  return preceding_lanes_vec.front();
}

std::vector<lanelet::ConstLanelet> RouteHandler::getLanesAfterGoal(
  const double vehicle_length) const
{
  lanelet::ConstLanelet goal_lanelet;
  if (!getGoalLanelet(&goal_lanelet)) {
    return std::vector<lanelet::ConstLanelet>{};
  }

  const double min_succeeding_length = vehicle_length * 2;
  const auto succeeding_lanes_vec = lanelet::utils::query::getSucceedingLaneletSequences(
    routing_graph_ptr_, goal_lanelet, min_succeeding_length);
  if (succeeding_lanes_vec.empty()) {
    return std::vector<lanelet::ConstLanelet>{};
  }

  return succeeding_lanes_vec.front();
}

lanelet::ConstLanelets RouteHandler::getRouteLanelets() const
{
  return route_lanelets_;
}

lanelet::ConstLanelets RouteHandler::getPreferredLanelets() const
{
  return preferred_lanelets_;
}

Pose RouteHandler::getStartPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getStartPose: Route has not been set yet");
    return Pose();
  }
  return route_ptr_->start_pose;
}

Pose RouteHandler::getOriginalStartPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getOriginalStartPose: Route has not been set yet");
    return Pose();
  }
  return original_start_pose_;
}

Pose RouteHandler::getGoalPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getGoalPose: Route has not been set yet");
    return Pose();
  }
  return route_ptr_->goal_pose;
}

Pose RouteHandler::getOriginalGoalPose() const
{
  if (!route_ptr_) {
    RCLCPP_WARN(logger_, "[Route Handler] getOriginalGoalPose: Route has not been set yet");
    return Pose();
  }
  return original_goal_pose_;
}

lanelet::Id RouteHandler::getGoalLaneId() const
{
  if (!route_ptr_ || route_ptr_->segments.empty()) {
    return lanelet::InvalId;
  }

  return route_ptr_->segments.back().preferred_primitive.id;
}

bool RouteHandler::getGoalLanelet(lanelet::ConstLanelet * goal_lanelet) const
{
  const lanelet::Id goal_lane_id = getGoalLaneId();
  for (const auto & llt : route_lanelets_) {
    if (llt.id() == goal_lane_id) {
      *goal_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::isInGoalRouteSection(const lanelet::ConstLanelet & lanelet) const
{
  if (!route_ptr_ || route_ptr_->segments.empty()) {
    return false;
  }
  return exists(route_ptr_->segments.back().primitives, lanelet.id());
}

lanelet::ConstLanelets RouteHandler::getLaneletsFromIds(const lanelet::Ids & ids) const
{
  lanelet::ConstLanelets lanelets;
  lanelets.reserve(ids.size());
  for (const auto & id : ids) {
    lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
  return lanelets;
}

lanelet::ConstLanelet RouteHandler::getLaneletsFromId(const lanelet::Id id) const
{
  return lanelet_map_ptr_->laneletLayer.get(id);
}

bool RouteHandler::isDeadEndLanelet(const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelet next_lanelet;
  return !getNextLaneletWithinRoute(lanelet, &next_lanelet);
}

lanelet::ConstLanelets RouteHandler::getLaneChangeableNeighbors(
  const lanelet::ConstLanelet & lanelet) const
{
  return lanelet::utils::query::getLaneChangeableNeighbors(routing_graph_ptr_, lanelet);
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length, const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      if (only_route_lanes) {
        break;
      }
      const auto next_lanes = getNextLanelets(current_lanelet);
      if (next_lanes.empty()) {
        break;
      }
      next_lanelet = next_lanes.front();
    }
    // loop check
    if (lanelet.id() == next_lanelet.id()) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(next_lanelet.centerline().basicLineString()));
  }

  return lanelet_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length, const bool only_route_lanes) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  double length = 0;
  lanelet::ConstLanelets previous_lanelets;

  auto checkForLoop =
    [&lanelet](const lanelet::ConstLanelets & lanelets_to_check, const bool is_route_lanelets) {
      if (is_route_lanelets) {
        return std::none_of(
          lanelets_to_check.begin(), lanelets_to_check.end(),
          [lanelet](auto & prev_llt) { return lanelet.id() != prev_llt.id(); });
      }
      return std::any_of(
        lanelets_to_check.begin(), lanelets_to_check.end(),
        [lanelet](auto & prev_llt) { return lanelet.id() == prev_llt.id(); });
    };

  auto isNewLanelet = [&lanelet,
                       &lanelet_sequence_backward](const lanelet::ConstLanelet & lanelet_to_check) {
    if (lanelet.id() == lanelet_to_check.id()) return false;
    return std::none_of(
      lanelet_sequence_backward.begin(), lanelet_sequence_backward.end(),
      [&lanelet_to_check](auto & backward) { return (backward.id() == lanelet_to_check.id()); });
  };

  while (rclcpp::ok() && length < min_length) {
    previous_lanelets.clear();
    bool is_route_lanelets = true;
    if (!getPreviousLaneletsWithinRoute(current_lanelet, &previous_lanelets)) {
      if (only_route_lanes) break;
      previous_lanelets = getPreviousLanelets(current_lanelet);
      if (previous_lanelets.empty()) break;
      is_route_lanelets = false;
    }

    if (checkForLoop(previous_lanelets, is_route_lanelets)) break;

    for (const auto & prev_lanelet : previous_lanelets) {
      if (!isNewLanelet(prev_lanelet) || exists(goal_lanelets_, prev_lanelet)) continue;
      lanelet_sequence_backward.push_back(prev_lanelet);
      length +=
        static_cast<double>(boost::geometry::length(prev_lanelet.centerline().basicLineString()));
      current_lanelet = prev_lanelet;
      break;
    }
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const double backward_distance,
  const double forward_distance, const bool only_route_lanes) const
{
  Pose current_pose{};
  current_pose.orientation.w = 1;
  if (!lanelet.centerline().empty()) {
    current_pose.position = lanelet::utils::conversion::toGeomMsgPt(lanelet.centerline().front());
  }

  lanelet::ConstLanelets lanelet_sequence;
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  const lanelet::ConstLanelets lanelet_sequence_forward = std::invoke([&]() {
    if (only_route_lanes) {
      return getLaneletSequenceAfter(lanelet, forward_distance, only_route_lanes);
    } else if (isShoulderLanelet(lanelet)) {
      return getShoulderLaneletSequenceAfter(lanelet, forward_distance);
    }
    return lanelet::ConstLanelets{};
  });
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      if (only_route_lanes) {
        return getLaneletSequenceUpTo(lanelet, backward_distance, only_route_lanes);
      } else if (isShoulderLanelet(lanelet)) {
        return getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
      }
    }
    return lanelet::ConstLanelets{};
  });

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & current_pose, const double backward_distance,
  const double forward_distance, const bool only_route_lanes) const
{
  if (only_route_lanes && !exists(route_lanelets_, lanelet)) {
    return {};
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getLaneletSequenceAfter(lanelet, forward_distance, only_route_lanes);
  lanelet::ConstLanelets lanelet_sequence = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      return getLaneletSequenceUpTo(lanelet, backward_distance, only_route_lanes);
    }
    return lanelet::ConstLanelets{};
  });

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence.empty()) {
    if (lanelet_sequence.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.push_back(lanelet);
  std::move(
    lanelet_sequence_forward.begin(), lanelet_sequence_forward.end(),
    std::back_inserter(lanelet_sequence));
  return lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getRoadLaneletsAtPose(const Pose & pose) const
{
  lanelet::ConstLanelets road_lanelets_at_pose;
  const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
  const auto lanelets_at_pose = lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p));
  for (const auto & lanelet_at_pose : lanelets_at_pose) {
    // confirm that the pose is inside the lanelet since "search" does an approximation with boxes
    const auto is_pose_inside_lanelet = lanelet::geometry::inside(lanelet_at_pose, p);
    if (is_pose_inside_lanelet && isRoadLanelet(lanelet_at_pose))
      road_lanelets_at_pose.push_back(lanelet_at_pose);
  }
  return road_lanelets_at_pose;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getFollowingShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  bool found = false;
  const auto & search_point = lanelet.centerline().back().basicPoint2d();
  const auto next_lanelet = lanelet_map_ptr_->laneletLayer.nearestUntil(
    search_point, [&](const auto & bbox, const auto & ll) {
      if (isShoulderLanelet(ll) && lanelet::geometry::follows(lanelet, ll)) found = true;
      // stop search once next shoulder lanelet is found, or the bbox does not touch the search
      // point
      return found || lanelet::geometry::distance2d(bbox, search_point) > 1e-3;
    });
  if (found && next_lanelet.has_value()) return *next_lanelet;
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLeftShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound())) {
    if (other_lanelet.rightBound() == lanelet.leftBound() && isShoulderLanelet(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getRightShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  for (const auto & other_lanelet :
       lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound())) {
    if (other_lanelet.leftBound() == lanelet.rightBound() && isShoulderLanelet(other_lanelet))
      return other_lanelet;
  }
  return std::nullopt;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletsAtPose(const Pose & pose) const
{
  lanelet::ConstLanelets lanelets_at_pose;
  const lanelet::BasicPoint2d p{pose.position.x, pose.position.y};
  const auto candidates_at_pose = lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p));
  for (const auto & candidate : candidates_at_pose) {
    // confirm that the pose is inside the lanelet since "search" does an approximation with boxes
    const auto is_pose_inside_lanelet = lanelet::geometry::inside(candidate, p);
    if (is_pose_inside_lanelet && isShoulderLanelet(candidate))
      lanelets_at_pose.push_back(candidate);
  }
  return lanelets_at_pose;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!isShoulderLanelet(lanelet)) return lanelet_sequence_forward;

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::set<lanelet::Id> searched_ids{};
  while (rclcpp::ok() && length < min_length) {
    const auto next_lanelet = getFollowingShoulderLanelet(current_lanelet);
    if (!next_lanelet) break;
    lanelet_sequence_forward.push_back(*next_lanelet);
    if (searched_ids.find(next_lanelet->id()) != searched_ids.end()) {
      // loop shoulder detected
      break;
    }
    searched_ids.insert(next_lanelet->id());
    current_lanelet = *next_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(next_lanelet->centerline().basicLineString()));
  }

  return lanelet_sequence_forward;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getPreviousShoulderLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  bool found = false;
  const auto & search_point = lanelet.centerline().front().basicPoint2d();
  const auto previous_lanelet = lanelet_map_ptr_->laneletLayer.nearestUntil(
    search_point, [&](const auto & bbox, const auto & ll) {
      if (isShoulderLanelet(ll) && lanelet::geometry::follows(ll, lanelet)) found = true;
      // stop search once prev shoulder lanelet is found, or the bbox does not touch the search
      // point
      return found || lanelet::geometry::distance2d(bbox, search_point) > 1e-3;
    });
  if (found && previous_lanelet.has_value()) return *previous_lanelet;
  return std::nullopt;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!isShoulderLanelet(lanelet)) return lanelet_sequence_backward;

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  std::set<lanelet::Id> searched_ids{};
  while (rclcpp::ok() && length < min_length) {
    const auto prev_lanelet = getPreviousShoulderLanelet(current_lanelet);
    if (!prev_lanelet) break;

    lanelet_sequence_backward.insert(lanelet_sequence_backward.begin(), *prev_lanelet);
    if (searched_ids.find(prev_lanelet->id()) != searched_ids.end()) {
      // loop shoulder detected
      break;
    }
    searched_ids.insert(prev_lanelet->id());
    current_lanelet = *prev_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(prev_lanelet->centerline().basicLineString()));
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & pose, const double backward_distance,
  const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  if (!isShoulderLanelet(lanelet)) {
    return lanelet_sequence;
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getShoulderLaneletSequenceAfter(lanelet, forward_distance);
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, pose);
    if (arc_coordinate.length < backward_distance) {
      return getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
    }
    return lanelet::ConstLanelets{};
  });

  // loop check
  if (!lanelet_sequence_forward.empty() && !lanelet_sequence_backward.empty()) {
    if (lanelet_sequence_backward.back().id() == lanelet_sequence_forward.front().id()) {
      return lanelet_sequence_forward;
    }
  }
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());

  lanelet_sequence.push_back(lanelet);
  lanelet_sequence.insert(
    lanelet_sequence.end(), lanelet_sequence_forward.begin(), lanelet_sequence_forward.end());

  return lanelet_sequence;
}

bool RouteHandler::getClosestLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  return lanelet::utils::query::getClosestLanelet(route_lanelets_, search_pose, closest_lanelet);
}

bool RouteHandler::getClosestPreferredLaneletWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet) const
{
  return lanelet::utils::query::getClosestLanelet(
    preferred_lanelets_, search_pose, closest_lanelet);
}

bool RouteHandler::getClosestLaneletWithConstrainsWithinRoute(
  const Pose & search_pose, lanelet::ConstLanelet * closest_lanelet, const double dist_threshold,
  const double yaw_threshold) const
{
  return lanelet::utils::query::getClosestLaneletWithConstrains(
    route_lanelets_, search_pose, closest_lanelet, dist_threshold, yaw_threshold);
}

bool RouteHandler::getClosestRouteLaneletFromLanelet(
  const Pose & search_pose, const lanelet::ConstLanelet & reference_lanelet,
  lanelet::ConstLanelet * closest_lanelet, const double dist_threshold,
  const double yaw_threshold) const
{
  lanelet::ConstLanelets previous_lanelets, next_lanelets, lanelet_sequence;
  if (getPreviousLaneletsWithinRoute(reference_lanelet, &previous_lanelets)) {
    lanelet_sequence = previous_lanelets;
  }

  lanelet_sequence.push_back(reference_lanelet);

  if (getNextLaneletsWithinRoute(reference_lanelet, &next_lanelets)) {
    lanelet_sequence.insert(lanelet_sequence.end(), next_lanelets.begin(), next_lanelets.end());
  }

  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        lanelet_sequence, search_pose, closest_lanelet, dist_threshold, yaw_threshold)) {
    return true;
  }

  return false;
}

bool RouteHandler::getNextLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * next_lanelets) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }

  const auto start_lane_id = route_ptr_->segments.front().preferred_primitive.id;

  const auto following_lanelets = routing_graph_ptr_->following(lanelet);
  next_lanelets->clear();
  for (const auto & llt : following_lanelets) {
    if (start_lane_id != llt.id() && exists(route_lanelets_, llt)) {
      next_lanelets->push_back(llt);
    }
  }
  return !(next_lanelets->empty());
}

bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  lanelet::ConstLanelets next_lanelets{};
  if (getNextLaneletsWithinRoute(lanelet, &next_lanelets)) {
    *next_lanelet = next_lanelets.front();
    return true;
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getNextLanelets(const lanelet::ConstLanelet & lanelet) const
{
  return routing_graph_ptr_->following(lanelet);
}

bool RouteHandler::getPreviousLaneletsWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelets * prev_lanelets) const
{
  if (exists(start_lanelets_, lanelet)) {
    return false;
  }
  const auto candidate_lanelets = routing_graph_ptr_->previous(lanelet);
  prev_lanelets->clear();
  for (const auto & llt : candidate_lanelets) {
    if (exists(route_lanelets_, llt)) {
      prev_lanelets->push_back(llt);
    }
  }
  return !(prev_lanelets->empty());
}

lanelet::ConstLanelets RouteHandler::getPreviousLanelets(
  const lanelet::ConstLanelet & lanelet) const
{
  return routing_graph_ptr_->previous(lanelet);
}

lanelet::ConstLanelets RouteHandler::getLaneletsFromPoint(const lanelet::ConstPoint3d & point) const
{
  return lanelet::utils::findUsagesInLanelets(*lanelet_map_ptr_, point);
}

bool RouteHandler::getRightLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet) const
{
  const auto opt_right_lanelet = routing_graph_ptr_->right(lanelet);
  if (!!opt_right_lanelet) {
    *right_lanelet = opt_right_lanelet.value();
    return exists(route_lanelets_, *right_lanelet);
  }
  return false;
}

bool RouteHandler::getNextLaneletWithinRouteExceptStart(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }
  const lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(lanelet);
  for (const auto & llt : following_lanelets) {
    if (exists(route_lanelets_, llt) && !exists(start_lanelets_, llt)) {
      *next_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getPreviousLaneletWithinRouteExceptGoal(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const
{
  if (exists(start_lanelets_, lanelet)) {
    return false;
  }
  const lanelet::ConstLanelets previous_lanelets = routing_graph_ptr_->previous(lanelet);
  for (const auto & llt : previous_lanelets) {
    if (exists(route_lanelets_, llt) && !(exists(goal_lanelets_, llt))) {
      *prev_lanelet = llt;
      return true;
    }
  }
  return false;
}

bool RouteHandler::isBijectiveConnection(
  const lanelet::ConstLanelets & lanelet_section1,
  const lanelet::ConstLanelets & lanelet_section2) const
{
  if (lanelet_section1.size() != lanelet_section2.size()) {
    return false;
  }

  // check injection
  for (const auto & lanelet : lanelet_section1) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRouteExceptStart(lanelet, &next_lanelet)) {
      return false;
    }
    if (!exists(lanelet_section2, next_lanelet)) {
      return false;
    }
  }

  // check surjection
  for (const auto & lanelet : lanelet_section2) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRouteExceptGoal(lanelet, &prev_lanelet)) {
      return false;
    }
    if (!exists(lanelet_section1, prev_lanelet)) {
      return false;
    }
  }
  return true;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getRightLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // right road lanelet of shoulder lanelet
  if (isShoulderLanelet(lanelet)) {
    const auto right_lanelets = lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound());
    for (const auto & right_lanelet : right_lanelets)
      if (isRoadLanelet(right_lanelet)) return right_lanelet;
    return std::nullopt;
  }

  // right shoulder lanelet
  if (get_shoulder_lane) {
    const auto right_shoulder_lanelet = getRightShoulderLanelet(lanelet);
    if (right_shoulder_lanelet) return *right_shoulder_lanelet;
  }

  // routable lane
  const auto & right_lane = routing_graph_ptr_->right(lanelet);
  if (right_lane) {
    return *right_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_right_lane = routing_graph_ptr_->adjacentRight(lanelet);
  if (adjacent_right_lane) {
    return *adjacent_right_lane;
  }

  // same root right lanelet
  if (!enable_same_root) {
    return std::nullopt;
  }

  lanelet::ConstLanelets prev_lanelet;
  if (!getPreviousLaneletsWithinRoute(lanelet, &prev_lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelet next_lanelet;
  if (!getNextLaneletWithinRoute(lanelet, &next_lanelet)) {
    for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
      if (lanelet.rightBound().back().id() == lane.leftBound().back().id()) {
        return lane;
      }
    }
    return std::nullopt;
  }

  const auto next_right_lane = getRightLanelet(next_lanelet, false);
  if (!next_right_lane) {
    return std::nullopt;
  }

  for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
    for (const auto & target_lane : getNextLanelets(lane)) {
      if (next_right_lane.value().id() == target_lane.id()) {
        return lane;
      }
    }
  }

  return std::nullopt;
}

bool RouteHandler::getLeftLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet) const
{
  const auto opt_left_lanelet = routing_graph_ptr_->left(lanelet);
  if (!!opt_left_lanelet) {
    *left_lanelet = opt_left_lanelet.value();
    return exists(route_lanelets_, *left_lanelet);
  }
  return false;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLeftLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // left road lanelet of shoulder lanelet
  if (isShoulderLanelet(lanelet)) {
    const auto left_lanelets = lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound());
    for (const auto & left_lanelet : left_lanelets)
      if (isRoadLanelet(left_lanelet)) return left_lanelet;
    return std::nullopt;
  }

  // left shoulder lanelet
  if (get_shoulder_lane) {
    const auto left_shoulder_lanelet = getLeftShoulderLanelet(lanelet);
    if (left_shoulder_lanelet) return *left_shoulder_lanelet;
  }

  // routable lane
  const auto & left_lane = routing_graph_ptr_->left(lanelet);
  if (left_lane) {
    return *left_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_left_lane = routing_graph_ptr_->adjacentLeft(lanelet);
  if (adjacent_left_lane) {
    return *adjacent_left_lane;
  }

  // same root right lanelet
  if (!enable_same_root) {
    return std::nullopt;
  }

  lanelet::ConstLanelets prev_lanelet;
  if (!getPreviousLaneletsWithinRoute(lanelet, &prev_lanelet)) {
    return std::nullopt;
  }

  lanelet::ConstLanelet next_lanelet;
  if (!getNextLaneletWithinRoute(lanelet, &next_lanelet)) {
    for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
      if (lanelet.leftBound().back().id() == lane.rightBound().back().id()) {
        return lane;
      }
    }
    return std::nullopt;
  }

  const auto next_left_lane = getLeftLanelet(next_lanelet, false);
  if (!next_left_lane) {
    return std::nullopt;
  }

  for (const auto & lane : getNextLanelets(prev_lanelet.front())) {
    for (const auto & target_lane : getNextLanelets(lane)) {
      if (next_left_lane.value().id() == target_lane.id()) {
        return lane;
      }
    }
  }

  return std::nullopt;
}

lanelet::Lanelets RouteHandler::getRightOppositeLanelets(
  const lanelet::ConstLanelet & lanelet) const
{
  const auto opposite_candidate_lanelets =
    lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.leftBound().id() == lanelet.rightBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

lanelet::ConstLanelets RouteHandler::getAllLeftSharedLinestringLanelets(
  const lanelet::ConstLanelet & lane, const bool & include_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets linestring_shared;
  auto lanelet_at_left = getLeftLanelet(lane);
  auto lanelet_at_left_opposite = getLeftOppositeLanelets(lane);
  while (lanelet_at_left) {
    linestring_shared.push_back(lanelet_at_left.value());
    lanelet_at_left = getLeftLanelet(lanelet_at_left.value());
    if (!lanelet_at_left) {
      break;
    }
    lanelet_at_left_opposite = getLeftOppositeLanelets(lanelet_at_left.value());
  }

  if (!lanelet_at_left_opposite.empty() && include_opposite) {
    if (invert_opposite) {
      linestring_shared.push_back(lanelet_at_left_opposite.front().invert());
    } else {
      linestring_shared.push_back(lanelet_at_left_opposite.front());
    }
    auto lanelet_at_right = getRightLanelet(lanelet_at_left_opposite.front());
    while (lanelet_at_right) {
      if (invert_opposite) {
        linestring_shared.push_back(lanelet_at_right.value().invert());
      } else {
        linestring_shared.push_back(lanelet_at_right.value());
      }
      lanelet_at_right = getRightLanelet(lanelet_at_right.value());
    }
  }
  return linestring_shared;
}

lanelet::ConstLanelets RouteHandler::getAllRightSharedLinestringLanelets(
  const lanelet::ConstLanelet & lane, const bool & include_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets linestring_shared;
  auto lanelet_at_right = getRightLanelet(lane);
  auto lanelet_at_right_opposite = getRightOppositeLanelets(lane);
  while (lanelet_at_right) {
    linestring_shared.push_back(lanelet_at_right.value());
    lanelet_at_right = getRightLanelet(lanelet_at_right.value());
    if (!lanelet_at_right) {
      break;
    }
    lanelet_at_right_opposite = getRightOppositeLanelets(lanelet_at_right.value());
  }

  if (!lanelet_at_right_opposite.empty() && include_opposite) {
    if (invert_opposite) {
      linestring_shared.push_back(lanelet_at_right_opposite.front().invert());
    } else {
      linestring_shared.push_back(lanelet_at_right_opposite.front());
    }
    auto lanelet_at_left = getLeftLanelet(lanelet_at_right_opposite.front());
    while (lanelet_at_left) {
      if (invert_opposite) {
        linestring_shared.push_back(lanelet_at_left.value().invert());
      } else {
        linestring_shared.push_back(lanelet_at_left.value());
      }
      lanelet_at_left = getLeftLanelet(lanelet_at_left.value());
    }
  }
  return linestring_shared;
}

lanelet::ConstLanelets RouteHandler::getAllSharedLineStringLanelets(
  const lanelet::ConstLanelet & current_lane, bool is_right, bool is_left, bool is_opposite,
  const bool & invert_opposite) const noexcept
{
  lanelet::ConstLanelets shared{current_lane};

  if (is_right) {
    const lanelet::ConstLanelets all_right_lanelets =
      getAllRightSharedLinestringLanelets(current_lane, is_opposite, invert_opposite);
    shared.insert(shared.end(), all_right_lanelets.begin(), all_right_lanelets.end());
  }

  if (is_left) {
    const lanelet::ConstLanelets all_left_lanelets =
      getAllLeftSharedLinestringLanelets(current_lane, is_opposite, invert_opposite);
    shared.insert(shared.end(), all_left_lanelets.begin(), all_left_lanelets.end());
  }

  return shared;
}

lanelet::Lanelets RouteHandler::getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet) const
{
  const auto opposite_candidate_lanelets =
    lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound().invert());

  lanelet::Lanelets opposite_lanelets;
  for (const auto & candidate_lanelet : opposite_candidate_lanelets) {
    if (candidate_lanelet.rightBound().id() == lanelet.leftBound().id()) {
      continue;
    }

    opposite_lanelets.push_back(candidate_lanelet);
  }

  return opposite_lanelets;
}

lanelet::ConstLanelet RouteHandler::getMostRightLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // recursively compute the width of the lanes
  const auto & same = getRightLanelet(lanelet, enable_same_root, get_shoulder_lane);

  if (same) {
    return getMostRightLanelet(same.value(), enable_same_root, get_shoulder_lane);
  }

  return lanelet;
}

lanelet::ConstLanelet RouteHandler::getMostLeftLanelet(
  const lanelet::ConstLanelet & lanelet, const bool enable_same_root,
  const bool get_shoulder_lane) const
{
  // recursively compute the width of the lanes
  const auto & same = getLeftLanelet(lanelet, enable_same_root, get_shoulder_lane);

  if (same) {
    return getMostLeftLanelet(same.value(), enable_same_root, get_shoulder_lane);
  }

  return lanelet;
}

std::vector<lanelet::ConstLanelets> RouteHandler::getPrecedingLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const double length,
  const lanelet::ConstLanelets & exclude_lanelets) const
{
  return lanelet::utils::query::getPrecedingLaneletSequences(
    routing_graph_ptr_, lanelet, length, exclude_lanelets);
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLaneChangeTarget(
  const lanelet::ConstLanelets & lanelets, const Direction direction) const
{
  for (const auto & lanelet : lanelets) {
    const int num = getNumLaneToPreferredLane(lanelet, direction);
    if (num == 0) {
      continue;
    }

    if (direction == Direction::NONE || direction == Direction::RIGHT) {
      if (num < 0) {
        const auto right_lanes = routing_graph_ptr_->right(lanelet);
        if (!!right_lanes) {
          return *right_lanes;
        }
      }
    }

    if (direction == Direction::NONE || direction == Direction::LEFT) {
      if (num > 0) {
        const auto left_lanes = routing_graph_ptr_->left(lanelet);
        if (!!left_lanes) {
          return *left_lanes;
        }
      }
    }
  }

  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getLaneChangeTargetExceptPreferredLane(
  const lanelet::ConstLanelets & lanelets, const Direction direction) const
{
  for (const auto & lanelet : lanelets) {
    if (direction == Direction::RIGHT) {
      // Get right lanelet if preferred lane is on the left
      if (getNumLaneToPreferredLane(lanelet, direction) < 0) {
        continue;
      }

      const auto right_lanes = routing_graph_ptr_->right(lanelet);
      if (!!right_lanes) {
        return *right_lanes;
      }
    }

    if (direction == Direction::LEFT) {
      // Get left lanelet if preferred lane is on the right
      if (getNumLaneToPreferredLane(lanelet, direction) > 0) {
        continue;
      }
      const auto left_lanes = routing_graph_ptr_->left(lanelet);
      if (!!left_lanes) {
        return *left_lanes;
      }
    }
  }

  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getPullOverTarget(const Pose & goal_pose) const
{
  const lanelet::BasicPoint2d p(goal_pose.position.x, goal_pose.position.y);
  constexpr auto search_distance = 0.1;
  const lanelet::BasicPoint2d offset(search_distance, search_distance);
  const auto lanelets_in_range =
    lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p - offset, p + offset));
  for (const auto & lanelet : lanelets_in_range) {
    const auto is_in_lanelet = lanelet::utils::isInLanelet(goal_pose, lanelet, search_distance);
    if (is_in_lanelet && isShoulderLanelet(lanelet)) return lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> RouteHandler::getPullOutStartLane(
  const Pose & pose, const double vehicle_width) const
{
  const lanelet::BasicPoint2d p(pose.position.x, pose.position.y);
  const auto search_distance = vehicle_width / 2.0;
  const lanelet::BasicPoint2d offset(search_distance, search_distance);
  const auto lanelets_in_range =
    lanelet_map_ptr_->laneletLayer.search(lanelet::BoundingBox2d(p - offset, p + offset));
  for (const auto & lanelet : lanelets_in_range) {
    const auto is_in_lanelet = lanelet::utils::isInLanelet(pose, lanelet, search_distance);
    if (is_in_lanelet && isShoulderLanelet(lanelet)) return lanelet;
  }
  return std::nullopt;
}

int RouteHandler::getNumLaneToPreferredLane(
  const lanelet::ConstLanelet & lanelet, const Direction direction) const
{
  if (exists(preferred_lanelets_, lanelet)) {
    return 0;
  }

  if ((direction == Direction::NONE) || (direction == Direction::RIGHT)) {
    int num{0};
    const auto & right_lanes =
      lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
    for (const auto & right : right_lanes) {
      num--;
      if (exists(preferred_lanelets_, right)) {
        return num;
      }
    }
  }

  if ((direction == Direction::NONE) || (direction == Direction::LEFT)) {
    const auto & left_lanes =
      lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
    int num = 0;
    for (const auto & left : left_lanes) {
      num++;
      if (exists(preferred_lanelets_, left)) {
        return num;
      }
    }
  }

  return 0;  // TODO(Horibe) check if return 0 is appropriate.
}

std::vector<double> RouteHandler::getLateralIntervalsToPreferredLane(
  const lanelet::ConstLanelet & lanelet, const Direction direction) const
{
  if (exists(preferred_lanelets_, lanelet)) {
    return {};
  }

  if ((direction == Direction::NONE) || (direction == Direction::RIGHT)) {
    std::vector<double> intervals;
    lanelet::ConstLanelet current_lanelet = lanelet;
    const auto & right_lanes =
      lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
    for (const auto & right : right_lanes) {
      const auto & current_centerline = current_lanelet.centerline();
      const auto & next_centerline = right.centerline();
      if (current_centerline.empty() || next_centerline.empty()) {
        return intervals;
      }
      const auto & curr_pt = current_centerline.front();
      const auto & next_pt = next_centerline.front();
      intervals.push_back(-lanelet::geometry::distance2d(to2D(curr_pt), to2D(next_pt)));

      if (exists(preferred_lanelets_, right)) {
        return intervals;
      }
      current_lanelet = right;
    }
  }

  if ((direction == Direction::NONE) || (direction == Direction::LEFT)) {
    std::vector<double> intervals;
    lanelet::ConstLanelet current_lanelet = lanelet;
    const auto & left_lanes =
      lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
    for (const auto & left : left_lanes) {
      const auto & current_centerline = current_lanelet.centerline();
      const auto & next_centerline = left.centerline();
      if (current_centerline.empty() || next_centerline.empty()) {
        return intervals;
      }
      const auto & curr_pt = current_centerline.front();
      const auto & next_pt = next_centerline.front();
      intervals.push_back(lanelet::geometry::distance2d(to2D(curr_pt), to2D(next_pt)));

      if (exists(preferred_lanelets_, left)) {
        return intervals;
      }
      current_lanelet = left;
    }
  }

  return {};
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  bool use_exact) const
{
  using lanelet::utils::to2D;
  using lanelet::utils::conversion::toLaneletPoint;

  // 1. calculate reference points by lanelets' centerline
  // NOTE: This vector aligns the vector lanelet_sequence.
  std::vector<PiecewiseReferencePoints> piecewise_ref_points_vec;
  const auto add_ref_point = [&](const auto & pt) {
    piecewise_ref_points_vec.back().push_back(
      ReferencePoint{false, lanelet::utils::conversion::toGeomMsgPt(pt)});
  };
  double s = 0;
  for (const auto & llt : lanelet_sequence) {
    piecewise_ref_points_vec.push_back(std::vector<ReferencePoint>{});

    const lanelet::ConstLineString3d centerline = llt.centerline();
    for (size_t i = 0; i < centerline.size(); i++) {
      const auto & pt = centerline[i];
      const lanelet::ConstPoint3d next_pt =
        (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      const double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_start) : pt;
        add_ref_point(p);
      }
      if (s >= s_start && s <= s_end) {
        add_ref_point(pt);
      }
      if (s < s_end && s + distance > s_end) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_end) : next_pt;
        add_ref_point(p);
      }
      s += distance;
    }
  }

  // 2. calculate waypoints
  const auto waypoints_vec = calcWaypointsVector(lanelet_sequence);

  // 3. remove points in the margin of the waypoint
  for (const auto & waypoints : waypoints_vec) {
    for (auto piecewise_waypoints_itr = waypoints.begin();
         piecewise_waypoints_itr != waypoints.end(); ++piecewise_waypoints_itr) {
      const auto & piecewise_waypoints = piecewise_waypoints_itr->piecewise_waypoints;
      const auto lanelet_id = piecewise_waypoints_itr->lanelet_id;

      // calculate index of lanelet_sequence which corresponds to piecewise_waypoints.
      const auto lanelet_sequence_itr = std::find_if(
        lanelet_sequence.begin(), lanelet_sequence.end(),
        [&](const auto & lanelet) { return lanelet.id() == lanelet_id; });
      if (lanelet_sequence_itr == lanelet_sequence.end()) {
        continue;
      }
      const size_t piecewise_waypoints_lanelet_sequence_index =
        std::distance(lanelet_sequence.begin(), lanelet_sequence_itr);

      // calculate reference points by waypoints
      const auto ref_points_by_waypoints = convertWaypointsToReferencePoints(piecewise_waypoints);

      // update reference points by waypoints
      const bool is_first_waypoint_contained = piecewise_waypoints_itr == waypoints.begin();
      const bool is_last_waypoint_contained = piecewise_waypoints_itr == waypoints.end() - 1;
      if (is_first_waypoint_contained || is_last_waypoint_contained) {
        // If piecewise_waypoints_itr is the end (first or last) of piecewise_waypoints

        const auto original_piecewise_ref_points =
          piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index);

        // define current_piecewise_ref_points, and initialize it with waypoints
        auto & current_piecewise_ref_points =
          piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index);
        current_piecewise_ref_points = ref_points_by_waypoints;
        if (is_first_waypoint_contained) {
          // add original reference points to current reference points, and remove reference points
          // overlapped with waypoints
          current_piecewise_ref_points.insert(
            current_piecewise_ref_points.begin(), original_piecewise_ref_points.begin(),
            original_piecewise_ref_points.end());
          const bool is_removing_direction_forward = false;
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence,
            piecewise_waypoints_lanelet_sequence_index, is_removing_direction_forward);
        }
        if (is_last_waypoint_contained) {
          // add original reference points to current reference points, and remove reference points
          // overlapped with waypoints
          current_piecewise_ref_points.insert(
            current_piecewise_ref_points.end(), original_piecewise_ref_points.begin(),
            original_piecewise_ref_points.end());
          const bool is_removing_direction_forward = true;
          removeOverlappedCenterlineWithWaypoints(
            piecewise_ref_points_vec, piecewise_waypoints, lanelet_sequence,
            piecewise_waypoints_lanelet_sequence_index, is_removing_direction_forward);
        }
      } else {
        // If piecewise_waypoints_itr is not the end (first or last) of piecewise_waypoints,
        // remove all the reference points and add waypoints.
        piecewise_ref_points_vec.at(piecewise_waypoints_lanelet_sequence_index) =
          ref_points_by_waypoints;
      }
    }
  }

  // 4. convert to PathPointsWithLaneIds
  PathWithLaneId reference_path{};
  for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
    const auto & lanelet = lanelet_sequence.at(lanelet_idx);
    const float speed_limit =
      static_cast<float>(traffic_rules_ptr_->speedLimit(lanelet).speedLimit.value());

    const auto & piecewise_ref_points = piecewise_ref_points_vec.at(lanelet_idx);
    for (const auto & ref_point : piecewise_ref_points) {
      PathPointWithLaneId p{};
      p.point.pose.position = ref_point.point;
      p.lane_ids.push_back(lanelet.id());
      p.point.longitudinal_velocity_mps = speed_limit;
      reference_path.points.push_back(p);
    }
  }
  reference_path = removeOverlappingPoints(reference_path);

  // append a point only when having one point so that yaw calculation would work
  if (reference_path.points.size() == 1) {
    const lanelet::Id lane_id = reference_path.points.front().lane_ids.front();
    const auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
    const auto point = reference_path.points.front().point.pose.position;
    const auto lane_yaw = lanelet::utils::getLaneletAngle(lanelet, point);
    PathPointWithLaneId path_point{};
    path_point.lane_ids.push_back(lane_id);
    constexpr double ds{0.1};
    path_point.point.pose.position.x = point.x + ds * std::cos(lane_yaw);
    path_point.point.pose.position.y = point.y + ds * std::sin(lane_yaw);
    path_point.point.pose.position.z = point.z;
    reference_path.points.push_back(path_point);
  }

  // set angle
  for (size_t i = 0; i < reference_path.points.size(); i++) {
    double angle{0.0};
    const auto & pts = reference_path.points;
    if (i + 1 < reference_path.points.size()) {
      angle = tier4_autoware_utils::calcAzimuthAngle(
        pts.at(i).point.pose.position, pts.at(i + 1).point.pose.position);
    } else if (i != 0) {
      angle = tier4_autoware_utils::calcAzimuthAngle(
        pts.at(i - 1).point.pose.position, pts.at(i).point.pose.position);
    }
    reference_path.points.at(i).point.pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(angle);
  }

  return reference_path;
}

std::vector<Waypoints> RouteHandler::calcWaypointsVector(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  std::vector<Waypoints> waypoints_vec;
  for (size_t lanelet_idx = 0; lanelet_idx < lanelet_sequence.size(); ++lanelet_idx) {
    const auto & lanelet = lanelet_sequence.at(lanelet_idx);
    if (!lanelet.hasAttribute("waypoints")) {
      continue;
    }

    // generate piecewise waypoints
    PiecewiseWaypoints piecewise_waypoints{lanelet.id(), {}};
    const auto waypoints_id = lanelet.attribute("waypoints").asId().value();
    for (const auto & waypoint : lanelet_map_ptr_->lineStringLayer.get(waypoints_id)) {
      piecewise_waypoints.piecewise_waypoints.push_back(
        lanelet::utils::conversion::toGeomMsgPt(waypoint));
    }
    if (piecewise_waypoints.piecewise_waypoints.empty()) {
      continue;
    }

    // check if the piecewise waypoints are connected to the previous piecewise waypoints
    if (
      !waypoints_vec.empty() && isClose(
                                  waypoints_vec.back().back().piecewise_waypoints.back(),
                                  piecewise_waypoints.piecewise_waypoints.front(), 1.0)) {
      waypoints_vec.back().push_back(piecewise_waypoints);
    } else {
      // add new waypoints
      Waypoints new_waypoints;
      new_waypoints.push_back(piecewise_waypoints);
      waypoints_vec.push_back(new_waypoints);
    }
  }

  return waypoints_vec;
}

void RouteHandler::removeOverlappedCenterlineWithWaypoints(
  std::vector<PiecewiseReferencePoints> & piecewise_ref_points_vec,
  const std::vector<geometry_msgs::msg::Point> & piecewise_waypoints,
  const lanelet::ConstLanelets & lanelet_sequence,
  const size_t piecewise_waypoints_lanelet_sequence_index,
  const bool is_removing_direction_forward) const
{
  const double waypoints_interpolation_arc_margin_ratio = 10.0;

  // calculate arc length threshold
  const double front_arc_length_threshold = [&]() {
    const auto front_waypoint_arc_coordinates = calcArcCoordinates(
      lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index), piecewise_waypoints.front());
    const double lanelet_arc_length = boost::geometry::length(
      lanelet::utils::to2D(lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index)
                             .centerline()
                             .basicLineString()));
    return -lanelet_arc_length + front_waypoint_arc_coordinates.length -
           std::abs(front_waypoint_arc_coordinates.distance) *
             waypoints_interpolation_arc_margin_ratio;
  }();
  const double back_arc_length_threshold = [&]() {
    const auto back_waypoint_arc_coordinates = calcArcCoordinates(
      lanelet_sequence.at(piecewise_waypoints_lanelet_sequence_index), piecewise_waypoints.back());
    return back_waypoint_arc_coordinates.length + std::abs(back_waypoint_arc_coordinates.distance) *
                                                    waypoints_interpolation_arc_margin_ratio;
  }();

  double offset_arc_length = 0.0;
  int target_lanelet_sequence_index = static_cast<int>(piecewise_waypoints_lanelet_sequence_index);
  while (isIndexWithinVector(lanelet_sequence, target_lanelet_sequence_index)) {
    auto & target_piecewise_ref_points = piecewise_ref_points_vec.at(target_lanelet_sequence_index);
    const double target_lanelet_arc_length = boost::geometry::length(lanelet::utils::to2D(
      lanelet_sequence.at(target_lanelet_sequence_index).centerline().basicLineString()));

    // search overlapped ref points in the target lanelet
    std::vector<size_t> overlapped_ref_points_indices{};
    const bool is_search_finished = [&]() {
      for (size_t ref_point_unsigned_index = 0;
           ref_point_unsigned_index < target_piecewise_ref_points.size();
           ++ref_point_unsigned_index) {
        const size_t ref_point_index =
          is_removing_direction_forward
            ? ref_point_unsigned_index
            : target_piecewise_ref_points.size() - 1 - ref_point_unsigned_index;
        const auto & ref_point = target_piecewise_ref_points.at(ref_point_index);

        // skip waypoints
        if (ref_point.is_waypoint) {
          if (
            target_lanelet_sequence_index ==
            static_cast<int>(piecewise_waypoints_lanelet_sequence_index)) {
            overlapped_ref_points_indices.clear();
          }
          continue;
        }

        const double ref_point_arc_length =
          (is_removing_direction_forward ? 0 : -target_lanelet_arc_length) +
          calcArcCoordinates(lanelet_sequence.at(target_lanelet_sequence_index), ref_point.point)
            .length;
        if (is_removing_direction_forward) {
          if (back_arc_length_threshold < offset_arc_length + ref_point_arc_length) {
            return true;
          }
        } else {
          if (offset_arc_length + ref_point_arc_length < front_arc_length_threshold) {
            return true;
          }
        }

        overlapped_ref_points_indices.push_back(ref_point_index);
      }
      return false;
    }();

    // remove overlapped indices from ref_points
    removeIndicesFromVector(target_piecewise_ref_points, overlapped_ref_points_indices);

    // break if searching overlapped centerline is finished.
    if (is_search_finished) {
      break;
    }

    target_lanelet_sequence_index += is_removing_direction_forward ? 1 : -1;
    offset_arc_length = (is_removing_direction_forward ? 1 : -1) * target_lanelet_arc_length;
  }
}

bool RouteHandler::isMapMsgReady() const
{
  return is_map_msg_ready_;
}

lanelet::routing::RoutingGraphPtr RouteHandler::getRoutingGraphPtr() const
{
  return routing_graph_ptr_;
}

lanelet::traffic_rules::TrafficRulesPtr RouteHandler::getTrafficRulesPtr() const
{
  return traffic_rules_ptr_;
}

std::shared_ptr<const lanelet::routing::RoutingGraphContainer> RouteHandler::getOverallGraphPtr()
  const
{
  return overall_graphs_ptr_;
}

lanelet::LaneletMapPtr RouteHandler::getLaneletMapPtr() const
{
  return lanelet_map_ptr_;
}

bool RouteHandler::isShoulderLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == "road_shoulder";
}

bool RouteHandler::isRouteLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet::utils::contains(route_lanelets_, lanelet);
}

bool RouteHandler::isRoadLanelet(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet.hasAttribute(lanelet::AttributeName::Subtype) &&
         lanelet.attribute(lanelet::AttributeName::Subtype) == lanelet::AttributeValueString::Road;
}

lanelet::ConstLanelets RouteHandler::getPreviousLaneletSequence(
  const lanelet::ConstLanelets & lanelet_sequence) const
{
  lanelet::ConstLanelets previous_lanelet_sequence;
  if (lanelet_sequence.empty()) {
    return previous_lanelet_sequence;
  }

  const auto & first_lane = lanelet_sequence.front();
  if (exists(start_lanelets_, first_lane)) {
    return previous_lanelet_sequence;
  }

  auto right_relations =
    lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, first_lane);
  for (const auto & right : right_relations) {
    previous_lanelet_sequence = getLaneletSequenceUpTo(right);
    if (!previous_lanelet_sequence.empty()) {
      return previous_lanelet_sequence;
    }
  }

  auto left_relations = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, first_lane);
  for (const auto & left : left_relations) {
    previous_lanelet_sequence = getLaneletSequenceUpTo(left);
    if (!previous_lanelet_sequence.empty()) {
      return previous_lanelet_sequence;
    }
  }
  return previous_lanelet_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneSequence(const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lane_sequence;
  const lanelet::ConstLanelets lane_sequence_up_to = getLaneSequenceUpTo(lanelet);
  const lanelet::ConstLanelets lane_sequence_after = getLaneSequenceAfter(lanelet);

  lane_sequence.insert(lane_sequence.end(), lane_sequence_up_to.begin(), lane_sequence_up_to.end());
  lane_sequence.insert(lane_sequence.end(), lane_sequence_after.begin(), lane_sequence_after.end());
  return lane_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneSequenceUpTo(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  lanelet::ConstLanelets candidate_lanelets;
  while (rclcpp::ok()) {
    candidate_lanelets.clear();
    if (!getPreviousLaneletsWithinRoute(current_lanelet, &candidate_lanelets)) {
      break;
    }

    // If lanelet_sequence_backward with input lanelet contains all candidate lanelets,
    // break the loop.
    if (std::all_of(
          candidate_lanelets.begin(), candidate_lanelets.end(),
          [lanelet_sequence_backward, lanelet](auto & prev_llt) {
            return std::any_of(
              lanelet_sequence_backward.begin(), lanelet_sequence_backward.end(),
              [prev_llt, lanelet](auto & llt) {
                return (llt.id() == prev_llt.id() || lanelet.id() == prev_llt.id());
              });
          })) {
      break;
    }

    lanelet::ConstLanelet prev_lanelet;
    for (const auto & prev_llt : candidate_lanelets) {
      if (std::any_of(
            lanelet_sequence_backward.begin(), lanelet_sequence_backward.end(),
            [prev_llt, lanelet](auto & llt) {
              return (llt.id() == prev_llt.id() || lanelet.id() == prev_llt.id());
            })) {
        continue;
      }
      prev_lanelet = prev_llt;
      break;
    }

    const lanelet::ConstLanelets current_lanelet_section = getNeighborsWithinRoute(current_lanelet);
    const lanelet::ConstLanelets prev_lanelet_section = getNeighborsWithinRoute(prev_lanelet);
    if (!isBijectiveConnection(prev_lanelet_section, current_lanelet_section)) {
      break;
    }
    lanelet_sequence_backward.push_back(prev_lanelet);
    current_lanelet = prev_lanelet;
  }

  std::reverse(lanelet_sequence_backward.begin(), lanelet_sequence_backward.end());
  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getLaneSequenceAfter(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lane_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lane_sequence_forward;
  }
  lane_sequence_forward.push_back(lanelet);

  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok()) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
      break;
    }

    const lanelet::ConstLanelets current_lanelet_section = getNeighborsWithinRoute(current_lanelet);
    const lanelet::ConstLanelets next_lanelet_section = getNeighborsWithinRoute(next_lanelet);
    if (!isBijectiveConnection(current_lanelet_section, next_lanelet_section)) {
      break;
    }
    lane_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
  }

  return lane_sequence_forward;
}

lanelet::ConstLanelets RouteHandler::getNeighborsWithinRoute(
  const lanelet::ConstLanelet & lanelet) const
{
  const lanelet::ConstLanelets neighbor_lanelets =
    lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, lanelet);
  lanelet::ConstLanelets neighbors_within_route;
  for (const auto & llt : neighbor_lanelets) {
    if (exists(route_lanelets_, llt)) {
      neighbors_within_route.push_back(llt);
    }
  }
  return neighbors_within_route;
}

bool RouteHandler::planPathLaneletsBetweenCheckpoints(
  const Pose & start_checkpoint, const Pose & goal_checkpoint,
  lanelet::ConstLanelets * path_lanelets, const bool consider_no_drivable_lanes) const
{
  // Find lanelets for start point. First, find all lanelets containing the start point to calculate
  // all possible route later. It fails when the point is not located on any road lanelet (e.g. the
  // start point is located out of any lanelets or road_shoulder lanelet which is not contained in
  // road lanelet). In that case, find the closest lanelet instead (within some maximum range).
  constexpr auto max_search_range = 20.0;
  auto start_lanelets = getRoadLaneletsAtPose(start_checkpoint);
  lanelet::ConstLanelet start_lanelet;
  if (start_lanelets.empty()) {
    const lanelet::BasicPoint2d p(start_checkpoint.position.x, start_checkpoint.position.y);
    const lanelet::BoundingBox2d bbox(
      lanelet::BasicPoint2d(p.x() - max_search_range, p.y() - max_search_range),
      lanelet::BasicPoint2d(p.x() + max_search_range, p.y() + max_search_range));
    // std::as_const(*ptr) to use the const version of the search function
    auto candidates = std::as_const(*lanelet_map_ptr_).laneletLayer.search(bbox);
    candidates.erase(
      std::remove_if(
        candidates.begin(), candidates.end(), [&](const auto & l) { return !isRoadLanelet(l); }),
      candidates.end());
    if (lanelet::utils::query::getClosestLanelet(candidates, start_checkpoint, &start_lanelet))
      start_lanelets = {start_lanelet};
  }
  if (start_lanelets.empty()) {
    RCLCPP_WARN_STREAM(
      logger_, "Failed to find current lanelet."
                 << std::endl
                 << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                 << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl);
    return false;
  }

  // Find lanelets for goal point.
  lanelet::ConstLanelet goal_lanelet;
  const lanelet::BasicPoint2d p(goal_checkpoint.position.x, goal_checkpoint.position.y);
  const lanelet::BoundingBox2d bbox(
    lanelet::BasicPoint2d(p.x() - max_search_range, p.y() - max_search_range),
    lanelet::BasicPoint2d(p.x() + max_search_range, p.y() + max_search_range));
  auto candidates = std::as_const(*lanelet_map_ptr_).laneletLayer.search(bbox);
  candidates.erase(
    std::remove_if(
      candidates.begin(), candidates.end(), [&](const auto & l) { return !isRoadLanelet(l); }),
    candidates.end());
  if (!lanelet::utils::query::getClosestLanelet(candidates, goal_checkpoint, &goal_lanelet)) {
    RCLCPP_WARN_STREAM(
      logger_, "Failed to find closest lanelet."
                 << std::endl
                 << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                 << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl);
    return false;
  }

  lanelet::Optional<lanelet::routing::Route> optional_route;
  std::vector<lanelet::ConstLanelets> candidate_paths;
  lanelet::routing::LaneletPath shortest_path;
  bool is_route_found = false;

  double smallest_angle_diff = std::numeric_limits<double>::max();
  constexpr double yaw_threshold = M_PI / 2.0;

  for (const auto & st_llt : start_lanelets) {
    // check if the angle difference between start_checkpoint and start lanelet center line
    // orientation is in yaw_threshold range
    double lanelet_angle = lanelet::utils::getLaneletAngle(st_llt, start_checkpoint.position);
    double pose_yaw = tf2::getYaw(start_checkpoint.orientation);
    double angle_diff = std::abs(autoware_utils::normalize_radian(lanelet_angle - pose_yaw));

    bool is_proper_angle = angle_diff <= std::abs(yaw_threshold);

    optional_route = routing_graph_ptr_->getRoute(st_llt, goal_lanelet, 0);
    if (!optional_route || !is_proper_angle) {
      RCLCPP_ERROR_STREAM(
        logger_, "Failed to find a proper route!"
                   << std::endl
                   << " - start checkpoint: " << toString(start_checkpoint) << std::endl
                   << " - goal checkpoint: " << toString(goal_checkpoint) << std::endl
                   << " - start lane id: " << st_llt.id() << std::endl
                   << " - goal lane id: " << goal_lanelet.id() << std::endl);
      continue;
    }
    is_route_found = true;
    if (angle_diff < smallest_angle_diff) {
      smallest_angle_diff = angle_diff;
      shortest_path = optional_route->shortestPath();
      start_lanelet = st_llt;
    }
  }

  if (is_route_found) {
    lanelet::routing::LaneletPath path;
    path = [&]() -> lanelet::routing::LaneletPath {
      if (consider_no_drivable_lanes && hasNoDrivableLaneInPath(shortest_path)) {
        const auto drivable_lane_path = findDrivableLanePath(start_lanelet, goal_lanelet);
        if (drivable_lane_path) return *drivable_lane_path;
      }
      return shortest_path;
    }();

    path_lanelets->reserve(path.size());
    for (const auto & llt : path) {
      path_lanelets->push_back(llt);
    }
  }

  return is_route_found;
}

std::vector<LaneletSegment> RouteHandler::createMapSegments(
  const lanelet::ConstLanelets & path_lanelets) const
{
  const auto main_path = getMainLanelets(path_lanelets);

  std::vector<LaneletSegment> route_sections;

  if (main_path.empty()) {
    return route_sections;
  }

  route_sections.reserve(main_path.size());
  for (const auto & main_llt : main_path) {
    LaneletSegment route_section_msg;
    const lanelet::ConstLanelets route_section_lanelets = getNeighborsWithinRoute(main_llt);
    route_section_msg.preferred_primitive.id = main_llt.id();
    route_section_msg.primitives.reserve(route_section_lanelets.size());
    for (const auto & section_llt : route_section_lanelets) {
      LaneletPrimitive p;
      p.id = section_llt.id();
      p.primitive_type = "lane";
      route_section_msg.primitives.push_back(p);
    }
    route_sections.push_back(route_section_msg);
  }
  return route_sections;
}

lanelet::ConstLanelets RouteHandler::getMainLanelets(
  const lanelet::ConstLanelets & path_lanelets) const
{
  auto lanelet_sequence = getLaneletSequence(path_lanelets.back());

  RCLCPP_INFO_STREAM(logger_, "getMainLanelets: lanelet_sequence = " << lanelet_sequence);

  lanelet::ConstLanelets main_lanelets;
  while (!lanelet_sequence.empty()) {
    main_lanelets.insert(main_lanelets.begin(), lanelet_sequence.begin(), lanelet_sequence.end());
    lanelet_sequence = getPreviousLaneletSequence(lanelet_sequence);
  }
  return main_lanelets;
}

bool RouteHandler::isNoDrivableLane(const lanelet::ConstLanelet & llt)
{
  const std::string no_drivable_lane_attribute = llt.attributeOr("no_drivable_lane", "no");
  return no_drivable_lane_attribute == "yes";
}

bool RouteHandler::hasNoDrivableLaneInPath(const lanelet::routing::LaneletPath & path) const
{
  for (const auto & llt : path)
    if (isNoDrivableLane(llt)) return true;
  return false;
}

std::optional<lanelet::routing::LaneletPath> RouteHandler::findDrivableLanePath(
  const lanelet::ConstLanelet & start_lanelet, const lanelet::ConstLanelet & goal_lanelet) const
{
  // we create a new routing graph with infinite cost on no drivable lanes
  const auto drivable_routing_graph_ptr = lanelet::routing::RoutingGraph::build(
    *lanelet_map_ptr_, *traffic_rules_ptr_,
    lanelet::routing::RoutingCostPtrs{std::make_shared<RoutingCostDrivable>()});
  const auto route = drivable_routing_graph_ptr->getRoute(start_lanelet, goal_lanelet, 0);
  if (route) return route->shortestPath();
  return {};
}
}  // namespace autoware::route_handler
