// Copyright 2021 Tier IV, Inc.
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

#include "route_handler/route_handler.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/route_checker.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletSequence.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletPrimitive;
using geometry_msgs::msg::Pose;
using lanelet::utils::to2D;

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
  filtered_path.drivable_area = input_path.drivable_area;
  return filtered_path;
}

std::string toString(const geometry_msgs::msg::Pose & pose)
{
  std::stringstream ss;
  ss << "(" << pose.position.x << ", " << pose.position.y << "," << pose.position.z << ")";
  return ss.str();
}

}  // namespace

namespace route_handler
{
RouteHandler::RouteHandler(const HADMapBin & map_msg) { setMap(map_msg); }

void RouteHandler::setMap(const HADMapBin & map_msg)
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
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);

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
    route_msg_ = route_msg;
    is_route_msg_ready_ = true;
    is_handler_ready_ = false;
    setLaneletsFromRouteMsg();
  } else {
    RCLCPP_ERROR(
      logger_,
      "Loop detected within route! Currently, no loop is allowed for route! Using previous route");
  }
}

bool RouteHandler::isHandlerReady() const { return is_handler_ready_; }

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

void RouteHandler::setLaneletsFromRouteMsg()
{
  if (!is_route_msg_ready_ || !is_map_msg_ready_) {
    return;
  }
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  const bool is_route_valid = lanelet::utils::route::isRouteValid(route_msg_, lanelet_map_ptr_);
  if (!is_route_valid) {
    return;
  }

  size_t primitive_size{0};
  for (const auto & route_section : route_msg_.segments) {
    primitive_size += route_section.primitives.size();
  }
  route_lanelets_.reserve(primitive_size);

  for (const auto & route_section : route_msg_.segments) {
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
  if (!route_msg_.segments.empty()) {
    goal_lanelets_.reserve(route_msg_.segments.back().primitives.size());
    for (const auto & primitive : route_msg_.segments.back().primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      goal_lanelets_.push_back(llt);
    }
    start_lanelets_.reserve(route_msg_.segments.front().primitives.size());
    for (const auto & primitive : route_msg_.segments.front().primitives) {
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

Header RouteHandler::getRouteHeader() const { return route_msg_.header; }

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

lanelet::ConstLanelets RouteHandler::getRouteLanelets() const { return route_lanelets_; }

Pose RouteHandler::getGoalPose() const { return route_msg_.goal_pose; }

lanelet::Id RouteHandler::getGoalLaneId() const
{
  if (route_msg_.segments.empty()) {
    return lanelet::InvalId;
  }

  return route_msg_.segments.back().preferred_primitive.id;
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
  if (route_msg_.segments.empty()) {
    return false;
  }
  return exists(route_msg_.segments.back().primitives, lanelet.id());
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

lanelet::ConstLanelets RouteHandler::getLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getNextLaneletWithinRoute(current_lanelet, &next_lanelet)) {
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
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  double length = 0;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelets candidate_lanelets;
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

    for (const auto & prev_lanelet : candidate_lanelets) {
      if (std::any_of(
            lanelet_sequence_backward.begin(), lanelet_sequence_backward.end(),
            [prev_lanelet, lanelet](auto & llt) {
              return (llt.id() == prev_lanelet.id() || lanelet.id() == prev_lanelet.id());
            })) {
        continue;
      }
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
  const double forward_distance) const
{
  Pose current_pose{};
  current_pose.orientation.w = 1;
  if (!lanelet.centerline().empty()) {
    current_pose.position = lanelet::utils::conversion::toGeomMsgPt(lanelet.centerline().front());
  }

  lanelet::ConstLanelets lanelet_sequence;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getLaneletSequenceAfter(lanelet, forward_distance);
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      return getLaneletSequenceUpTo(lanelet, backward_distance);
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
  const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet::ConstLanelets lanelet_sequence_forward =
    getLaneletSequenceAfter(lanelet, forward_distance);
  const lanelet::ConstLanelets lanelet_sequence_backward = std::invoke([&]() {
    const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
    if (arc_coordinate.length < backward_distance) {
      return getLaneletSequenceUpTo(lanelet, backward_distance);
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

bool RouteHandler::getFollowingShoulderLanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * following_lanelet) const
{
  Pose back_pose;
  back_pose.position.x = lanelet.centerline2d().back().x();
  back_pose.position.y = lanelet.centerline2d().back().y();
  back_pose.position.z = 0;

  for (const auto & shoulder_lanelet : shoulder_lanelets_) {
    Pose front_pose;
    front_pose.position.x = shoulder_lanelet.centerline2d().front().x();
    front_pose.position.y = shoulder_lanelet.centerline2d().front().y();
    front_pose.position.z = 0;
    if (
      std::hypot(
        front_pose.position.x - back_pose.position.x,
        front_pose.position.y - back_pose.position.y) < 5) {
      *following_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceAfter(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(shoulder_lanelets_, lanelet)) {
    return lanelet_sequence_forward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet next_lanelet;
    if (!getFollowingShoulderLanelet(current_lanelet, &next_lanelet)) {
      break;
    }
    lanelet_sequence_forward.push_back(next_lanelet);
    current_lanelet = next_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(next_lanelet.centerline().basicLineString()));
  }

  return lanelet_sequence_forward;
}

bool RouteHandler::getPreviousShoulderLanelet(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const
{
  Pose front_pose;
  front_pose.position.x = lanelet.centerline2d().front().x();
  front_pose.position.y = lanelet.centerline2d().front().y();
  front_pose.position.z = 0;

  for (const auto & shoulder_lanelet : shoulder_lanelets_) {
    Pose back_pose;
    back_pose.position.x = shoulder_lanelet.centerline2d().back().x();
    back_pose.position.y = shoulder_lanelet.centerline2d().back().y();
    back_pose.position.z = 0;
    if (
      std::hypot(
        front_pose.position.x - back_pose.position.x,
        front_pose.position.y - back_pose.position.y) < 5) {
      *prev_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequenceUpTo(
  const lanelet::ConstLanelet & lanelet, const double min_length) const
{
  lanelet::ConstLanelets lanelet_sequence_backward;
  if (!exists(shoulder_lanelets_, lanelet)) {
    return lanelet_sequence_backward;
  }

  double length = 0;
  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok() && length < min_length) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousShoulderLanelet(current_lanelet, &prev_lanelet)) {
      break;
    }

    lanelet_sequence_backward.insert(lanelet_sequence_backward.begin(), prev_lanelet);
    current_lanelet = prev_lanelet;
    length +=
      static_cast<double>(boost::geometry::length(prev_lanelet.centerline().basicLineString()));
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & pose, const double backward_distance,
  const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  if (!exists(shoulder_lanelets_, lanelet)) {
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

bool RouteHandler::getNextLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }
  const auto following_lanelets = routing_graph_ptr_->following(lanelet);
  for (const auto & llt : following_lanelets) {
    if (exists(route_lanelets_, llt)) {
      *next_lanelet = llt;
      return true;
    }
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
    *right_lanelet = opt_right_lanelet.get();
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

boost::optional<lanelet::ConstLanelet> RouteHandler::getRightLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  // routable lane
  const auto & right_lane = routing_graph_ptr_->right(lanelet);
  if (right_lane) {
    return right_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_right_lane = routing_graph_ptr_->adjacentRight(lanelet);
  return adjacent_right_lane;
}

bool RouteHandler::getLeftLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * left_lanelet) const
{
  const auto opt_left_lanelet = routing_graph_ptr_->left(lanelet);
  if (!!opt_left_lanelet) {
    *left_lanelet = opt_left_lanelet.get();
    return exists(route_lanelets_, *left_lanelet);
  }
  return false;
}

boost::optional<lanelet::ConstLanelet> RouteHandler::getLeftLanelet(
  const lanelet::ConstLanelet & lanelet) const
{
  // routable lane
  const auto & left_lane = routing_graph_ptr_->left(lanelet);
  if (left_lane) {
    return left_lane;
  }

  // non-routable lane (e.g. lane change infeasible)
  const auto & adjacent_left_lane = routing_graph_ptr_->adjacentLeft(lanelet);
  return adjacent_left_lane;
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
    linestring_shared.push_back(lanelet_at_left.get());
    lanelet_at_left = getLeftLanelet(lanelet_at_left.get());
    if (!lanelet_at_left) {
      break;
    }
    lanelet_at_left_opposite = getLeftOppositeLanelets(lanelet_at_left.get());
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
        linestring_shared.push_back(lanelet_at_right.get().invert());
      } else {
        linestring_shared.push_back(lanelet_at_right.get());
      }
      lanelet_at_right = getRightLanelet(lanelet_at_right.get());
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
    linestring_shared.push_back(lanelet_at_right.get());
    lanelet_at_right = getRightLanelet(lanelet_at_right.get());
    if (!lanelet_at_right) {
      break;
    }
    lanelet_at_right_opposite = getRightOppositeLanelets(lanelet_at_right.get());
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
        linestring_shared.push_back(lanelet_at_left.get().invert());
      } else {
        linestring_shared.push_back(lanelet_at_left.get());
      }
      lanelet_at_left = getLeftLanelet(lanelet_at_left.get());
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

lanelet::ConstLineString3d RouteHandler::getRightMostSameDirectionLinestring(
  const lanelet::ConstLanelet & lanelet) const noexcept
{
  // recursively compute the width of the lanes
  const auto & same = getRightLanelet(lanelet);

  if (same) {
    return getRightMostSameDirectionLinestring(same.get());
  }
  return lanelet.rightBound();
}

lanelet::ConstLineString3d RouteHandler::getRightMostLinestring(
  const lanelet::ConstLanelet & lanelet) const noexcept
{
  const auto & same = getRightLanelet(lanelet);
  const auto & opposite = getRightOppositeLanelets(lanelet);
  if (!same && opposite.empty()) {
    return lanelet.rightBound();
  }

  if (same) {
    return getRightMostLinestring(same.get());
  }

  if (!opposite.empty()) {
    return getLeftMostLinestring(lanelet::ConstLanelet(opposite.front()));
  }

  return lanelet.rightBound();
}

lanelet::ConstLineString3d RouteHandler::getLeftMostSameDirectionLinestring(
  const lanelet::ConstLanelet & lanelet) const noexcept
{
  // recursively compute the width of the lanes
  const auto & same = getLeftLanelet(lanelet);

  if (same) {
    return getLeftMostSameDirectionLinestring(same.get());
  }
  return lanelet.leftBound();
}

lanelet::ConstLineString3d RouteHandler::getLeftMostLinestring(
  const lanelet::ConstLanelet & lanelet) const noexcept
{
  // recursively compute the width of the lanes
  const auto & same = getLeftLanelet(lanelet);
  const auto & opposite = getLeftOppositeLanelets(lanelet);

  if (!same && opposite.empty()) {
    return lanelet.leftBound();
  }

  if (same) {
    return getLeftMostLinestring(same.get());
  }

  if (!opposite.empty()) {
    return getRightMostLinestring(lanelet::ConstLanelet(opposite.front()));
  }

  return lanelet.leftBound();
}

lanelet::ConstLineStrings3d RouteHandler::getFurthestLinestring(
  const lanelet::ConstLanelet & lanelet, bool is_right, bool is_left,
  bool is_opposite) const noexcept
{
  lanelet::ConstLineStrings3d linestrings;
  linestrings.reserve(2);

  if (is_right && is_opposite) {
    linestrings.emplace_back(getRightMostLinestring(lanelet));
  } else if (is_right && !is_opposite) {
    linestrings.emplace_back(getRightMostSameDirectionLinestring(lanelet));
  } else {
    linestrings.emplace_back(lanelet.rightBound());
  }

  if (is_left && is_opposite) {
    linestrings.emplace_back(getLeftMostLinestring(lanelet));
  } else if (is_left && !is_opposite) {
    linestrings.emplace_back(getLeftMostSameDirectionLinestring(lanelet));
  } else {
    linestrings.emplace_back(lanelet.leftBound());
  }
  return linestrings;
}

bool RouteHandler::getLaneChangeTarget(
  const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const
{
  for (const auto & lanelet : lanelets) {
    const int num = getNumLaneToPreferredLane(lanelet);
    if (num == 0) {
      continue;
    }

    if (num < 0) {
      if (!!routing_graph_ptr_->right(lanelet)) {
        const auto right_lanelet = routing_graph_ptr_->right(lanelet);
        *target_lanelet = right_lanelet.get();
        return true;
      }
      continue;
    }

    if (num > 0) {
      if (!!routing_graph_ptr_->left(lanelet)) {
        const auto left_lanelet = routing_graph_ptr_->left(lanelet);
        *target_lanelet = left_lanelet.get();
        return true;
      }
      continue;
    }
  }

  *target_lanelet = lanelets.front();
  return false;
}

bool RouteHandler::getPullOverTarget(
  const lanelet::ConstLanelets & lanelets, const Pose & goal_pose,
  lanelet::ConstLanelet * target_lanelet)
{
  for (const auto & shoulder_lanelet : lanelets) {
    if (lanelet::utils::isInLanelet(goal_pose, shoulder_lanelet, 0.1)) {
      *target_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getPullOutStartLane(
  const lanelet::ConstLanelets & lanelets, const Pose & pose, const double vehicle_width,
  lanelet::ConstLanelet * target_lanelet)
{
  for (const auto & shoulder_lanelet : lanelets) {
    if (lanelet::utils::isInLanelet(pose, shoulder_lanelet, vehicle_width / 2.0)) {
      *target_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getClosestLaneletSequence(const Pose & pose) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose, &lanelet)) {
    return lanelet::ConstLanelets{};
  }
  return getLaneletSequence(lanelet);
}

int RouteHandler::getNumLaneToPreferredLane(const lanelet::ConstLanelet & lanelet) const
{
  int num = 0;
  if (exists(preferred_lanelets_, lanelet)) {
    return num;
  }
  const auto & right_lanes =
    lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_, lanelet);
  for (const auto & right : right_lanes) {
    num--;
    if (exists(preferred_lanelets_, right)) {
      return num;
    }
  }
  const auto & left_lanes = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_, lanelet);
  num = 0;
  for (const auto & left : left_lanes) {
    num++;
    if (exists(preferred_lanelets_, left)) {
      return num;
    }
  }

  return 0;  // TODO(Horibe) check if return 0 is appropriate.
}

bool RouteHandler::isInPreferredLane(const PoseStamped & pose) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose.pose, &lanelet)) {
    return false;
  }
  return exists(preferred_lanelets_, lanelet);
}
bool RouteHandler::isInTargetLane(
  const PoseStamped & pose, const lanelet::ConstLanelets & target) const
{
  lanelet::ConstLanelet lanelet;
  if (!getClosestLaneletWithinRoute(pose.pose, &lanelet)) {
    return false;
  }
  return exists(target, lanelet);
}

PathWithLaneId RouteHandler::getCenterLinePath(
  const lanelet::ConstLanelets & lanelet_sequence, const double s_start, const double s_end,
  bool use_exact) const
{
  PathWithLaneId reference_path{};
  double s = 0;

  for (const auto & llt : lanelet_sequence) {
    const lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    const lanelet::ConstLineString3d centerline = llt.centerline();

    const auto add_path_point = [&reference_path, &limit, &llt](const auto & pt) {
      PathPointWithLaneId p{};
      p.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
      p.lane_ids.push_back(llt.id());
      p.point.longitudinal_velocity_mps = static_cast<float>(limit.speedLimit.value());
      reference_path.points.push_back(p);
    };

    for (size_t i = 0; i < centerline.size(); i++) {
      const auto & pt = centerline[i];
      const lanelet::ConstPoint3d next_pt =
        (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      const double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_start) : pt;
        add_path_point(p);
      }
      if (s >= s_start && s <= s_end) {
        add_path_point(pt);
      }
      if (s < s_end && s + distance > s_end) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_end) : next_pt;
        add_path_point(p);
      }
      s += distance;
    }
  }

  reference_path = removeOverlappingPoints(reference_path);

  // append a point only when having one point so that yaw calculation would work
  if (reference_path.points.size() == 1) {
    const int lane_id = static_cast<int>(reference_path.points.front().lane_ids.front());
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

PathWithLaneId RouteHandler::updatePathTwist(const PathWithLaneId & path) const
{
  PathWithLaneId updated_path = path;
  for (auto & point : updated_path.points) {
    const auto id = point.lane_ids.at(0);
    const auto llt = lanelet_map_ptr_->laneletLayer.get(id);
    const auto limit = traffic_rules_ptr_->speedLimit(llt);
    point.point.longitudinal_velocity_mps = static_cast<float>(limit.speedLimit.value());
  }
  return updated_path;
}

lanelet::ConstLanelets RouteHandler::getLaneChangeTargetLanes(const Pose & pose) const
{
  lanelet::ConstLanelet lanelet;
  lanelet::ConstLanelets target_lanelets;
  if (!getClosestLaneletWithinRoute(pose, &lanelet)) {
    return target_lanelets;
  }

  const int num = getNumLaneToPreferredLane(lanelet);
  if (num < 0) {
    const auto right_lanelet = (!!routing_graph_ptr_->right(lanelet))
                                 ? routing_graph_ptr_->right(lanelet)
                                 : routing_graph_ptr_->adjacentRight(lanelet);
    target_lanelets = getLaneletSequence(right_lanelet.get());
  }
  if (num > 0) {
    const auto left_lanelet = (!!routing_graph_ptr_->left(lanelet))
                                ? routing_graph_ptr_->left(lanelet)
                                : routing_graph_ptr_->adjacentLeft(lanelet);
    target_lanelets = getLaneletSequence(left_lanelet.get());
  }
  return target_lanelets;
}

double RouteHandler::getLaneChangeableDistance(
  const Pose & current_pose, const LaneChangeDirection & direction) const
{
  lanelet::ConstLanelet current_lane;
  if (!getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    return 0;
  }

  // get lanelets after current lane
  auto lanelet_sequence = getLaneletSequenceAfter(current_lane);
  lanelet_sequence.insert(lanelet_sequence.begin(), current_lane);

  double accumulated_distance = 0;
  for (const auto & lane : lanelet_sequence) {
    lanelet::ConstLanelet target_lane;
    if (direction == LaneChangeDirection::RIGHT) {
      if (!getRightLaneletWithinRoute(lane, &target_lane)) {
        break;
      }
    }
    if (direction == LaneChangeDirection::LEFT) {
      if (!getLeftLaneletWithinRoute(lane, &target_lane)) {
        break;
      }
    }
    double lane_length = lanelet::utils::getLaneletLength3d(lane);

    // overwrite  goal because lane change must be finished before reaching goal
    if (isInGoalRouteSection(lane)) {
      const auto goal_position = lanelet::utils::conversion::toLaneletPoint(getGoalPose().position);
      const auto goal_arc_coordinates = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(lane.centerline()), lanelet::utils::to2D(goal_position).basicPoint());
      lane_length = std::min(goal_arc_coordinates.length, lane_length);
    }

    // subtract distance up to current position for first lane
    if (lane == current_lane) {
      const auto current_position =
        lanelet::utils::conversion::toLaneletPoint(current_pose.position);
      const auto arc_coordinate = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(lane.centerline()),
        lanelet::utils::to2D(current_position).basicPoint());
      lane_length = std::max(lane_length - arc_coordinate.length, 0.0);
    }
    accumulated_distance += lane_length;
  }
  return accumulated_distance;
}

lanelet::ConstLanelets RouteHandler::getCheckTargetLanesFromPath(
  const PathWithLaneId & path, const lanelet::ConstLanelets & target_lanes,
  const double check_length) const
{
  std::vector<int64_t> target_lane_ids;
  target_lane_ids.reserve(target_lanes.size());
  for (const auto & llt : target_lanes) {
    target_lane_ids.push_back(llt.id());
  }

  // find first lanelet in target lanes along path
  int64_t root_lane_id = lanelet::InvalId;
  for (const auto & point : path.points) {
    for (const auto & lane_id : point.lane_ids) {
      if (exists(target_lane_ids, lane_id)) {
        root_lane_id = lane_id;
      }
    }
  }
  // return emtpy lane if failed to find root lane_id
  if (root_lane_id == lanelet::InvalId) {
    return target_lanes;
  }
  lanelet::ConstLanelet root_lanelet;
  for (const auto & llt : target_lanes) {
    if (llt.id() == root_lane_id) {
      root_lanelet = llt;
    }
  }

  const auto sequences = lanelet::utils::query::getPrecedingLaneletSequences(
    routing_graph_ptr_, root_lanelet, check_length);
  lanelet::ConstLanelets check_lanelets;
  for (const auto & sequence : sequences) {
    for (const auto & llt : sequence) {
      if (!lanelet::utils::contains(check_lanelets, llt)) {
        check_lanelets.push_back(llt);
      }
    }
  }
  for (const auto & llt : target_lanes) {
    if (!lanelet::utils::contains(check_lanelets, llt)) {
      check_lanelets.push_back(llt);
    }
  }
  return check_lanelets;
}

bool RouteHandler::isMapMsgReady() const { return is_map_msg_ready_; }

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

lanelet::LaneletMapPtr RouteHandler::getLaneletMapPtr() const { return lanelet_map_ptr_; }

lanelet::routing::RelationType RouteHandler::getRelation(
  const lanelet::ConstLanelet & prev_lane, const lanelet::ConstLanelet & next_lane) const
{
  if (prev_lane == next_lane) {
    return lanelet::routing::RelationType::None;
  }
  const auto & relation = routing_graph_ptr_->routingRelation(prev_lane, next_lane);
  if (relation) {
    return relation.get();
  }

  // check if lane change extends across multiple lanes
  const auto shortest_path = routing_graph_ptr_->shortestPath(prev_lane, next_lane);
  if (shortest_path) {
    auto prev_llt = shortest_path->front();
    for (const auto & llt : shortest_path.get()) {
      if (prev_llt == llt) {
        continue;
      }
      const auto & relation = routing_graph_ptr_->routingRelation(prev_llt, llt);
      if (!relation) {
        continue;
      }
      if (
        relation.get() == lanelet::routing::RelationType::Left ||
        relation.get() == lanelet::routing::RelationType::Right) {
        return relation.get();
      }
      prev_llt = llt;
    }
  }

  return lanelet::routing::RelationType::None;
}

lanelet::ConstLanelets RouteHandler::getShoulderLanelets() const { return shoulder_lanelets_; }

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
  while (rclcpp::ok()) {
    lanelet::ConstLanelets candidate_lanelets;
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

std::vector<lanelet::ConstLanelets> RouteHandler::getLaneSection(
  const lanelet::ConstLanelet & lanelet) const
{
  const lanelet::ConstLanelets neighbors = getNeighborsWithinRoute(lanelet);
  std::vector<lanelet::ConstLanelets> lane_section;
  lane_section.reserve(neighbors.size());
  for (const auto & llt : neighbors) {
    lane_section.push_back(getLaneSequence(llt));
  }
  return lane_section;
}

lanelet::ConstLanelets RouteHandler::getNextLaneSequence(
  const lanelet::ConstLanelets & lane_sequence) const
{
  lanelet::ConstLanelets next_lane_sequence;
  if (lane_sequence.empty()) {
    return next_lane_sequence;
  }
  const auto & final_lanelet = lane_sequence.back();
  lanelet::ConstLanelet next_lanelet;
  if (!getNextLaneletWithinRoute(final_lanelet, &next_lanelet)) {
    return next_lane_sequence;
  }
  return getLaneSequence(next_lanelet);
}

bool RouteHandler::planPathLaneletsBetweenCheckpoints(
  const Pose & start_checkpoint, const Pose & goal_checkpoint,
  lanelet::ConstLanelets * path_lanelets) const
{
  lanelet::Lanelet start_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, start_checkpoint, &start_lanelet)) {
    return false;
  }
  lanelet::Lanelet goal_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, goal_checkpoint, &goal_lanelet)) {
    return false;
  }

  // get all possible lanes that can be used to reach goal (including all possible lane change)
  const lanelet::Optional<lanelet::routing::Route> optional_route =
    routing_graph_ptr_->getRoute(start_lanelet, goal_lanelet, 0);
  if (!optional_route) {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to find a proper path!"
                 << std::endl
                 << "start checkpoint: " << toString(start_checkpoint) << std::endl
                 << "goal checkpoint: " << toString(goal_checkpoint) << std::endl
                 << "start lane id: " << start_lanelet.id() << std::endl
                 << "goal lane id: " << goal_lanelet.id() << std::endl);
    return false;
  }

  const auto shortest_path = optional_route->shortestPath();
  path_lanelets->reserve(shortest_path.size());
  for (const auto & llt : shortest_path) {
    path_lanelets->push_back(llt);
  }
  return true;
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
  lanelet::ConstLanelets main_lanelets;
  while (!lanelet_sequence.empty()) {
    main_lanelets.insert(main_lanelets.begin(), lanelet_sequence.begin(), lanelet_sequence.end());
    lanelet_sequence = getPreviousLaneletSequence(lanelet_sequence);
  }
  return main_lanelets;
}

}  // namespace route_handler
