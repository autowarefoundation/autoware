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
using autoware_auto_mapping_msgs::msg::MapPrimitive;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using lanelet::utils::to2D;

bool exists(const std::vector<MapPrimitive> & primitives, const int64_t & id)
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

bool isRouteLooped(const autoware_auto_planning_msgs::msg::HADMapRoute & route_msg)
{
  const auto & route_sections = route_msg.segments;
  for (const auto & route_section : route_sections) {
    const auto primitives = route_section.primitives;
    for (auto itr = primitives.begin(); itr != primitives.end(); ++itr) {
      const auto next_itr = itr + 1;
      if (next_itr == primitives.end()) break;
      if (std::any_of(next_itr, primitives.end(), [itr](auto p) { return p.id == itr->id; })) {
        return true;
      }
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
      double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
      if (accumulated_distance2d + distance2d > s) {
        double ratio = (s - accumulated_distance2d) / distance2d;
        auto interpolated_pt = prev_pt.basicPoint() * (1 - ratio) + pt.basicPoint() * ratio;
        return lanelet::ConstPoint3d(
          lanelet::InvalId, interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z());
      }
      accumulated_distance2d += distance2d;
      prev_pt = pt;
    }
  }
  return lanelet::ConstPoint3d();
}

Path convertToPathFromPathWithLaneId(const PathWithLaneId & path_with_lane_id)
{
  Path path;
  path.header = path_with_lane_id.header;
  path.drivable_area = path_with_lane_id.drivable_area;
  for (const auto & pt_with_lane_id : path_with_lane_id.points) {
    path.points.push_back(pt_with_lane_id.point);
  }
  return path;
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

double getDistanceToShoulderBoundary(
  const lanelet::ConstLanelets & shoulder_lanelets, const Pose & pose)
{
  lanelet::ConstLanelet closest_shoulder_lanelet;
  lanelet::ArcCoordinates arc_coordinates;
  if (lanelet::utils::query::getClosestLanelet(
        shoulder_lanelets, pose, &closest_shoulder_lanelet)) {
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(pose.position);
    const auto & left_line_2d = lanelet::utils::to2D(closest_shoulder_lanelet.leftBound3d());
    arc_coordinates = lanelet::geometry::toArcCoordinates(
      left_line_2d, lanelet::utils::to2D(lanelet_point).basicPoint());

  } else {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("utilities"),
      "closest shoulder lanelet not found.");
  }

  return arc_coordinates.distance;
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
  lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);

  is_map_msg_ready_ = true;
  is_handler_ready_ = false;

  setLaneletsFromRouteMsg();
}

void RouteHandler::setRoute(const HADMapRoute & route_msg)
{
  if (!isRouteLooped(route_msg)) {
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
    auto first_lanelet = path_lanelets.front();
    start_lanelets_ = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, first_lanelet);
    auto last_lanelet = path_lanelets.back();
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
  for (const auto & id : route_lanelets_id) {
    route_lanelets_.push_back(lanelet_map_ptr_->laneletLayer.get(id));
  }
}

void RouteHandler::setLaneletsFromRouteMsg()
{
  if (!is_route_msg_ready_ || !is_map_msg_ready_) {
    return;
  }
  route_lanelets_.clear();
  preferred_lanelets_.clear();
  for (const auto & route_section : route_msg_.segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      route_lanelets_.push_back(llt);
      if (id == route_section.preferred_primitive_id) {
        preferred_lanelets_.push_back(llt);
      }
    }
  }
  goal_lanelets_.clear();
  start_lanelets_.clear();
  if (!route_msg_.segments.empty()) {
    for (const auto & primitive : route_msg_.segments.back().primitives) {
      const auto id = primitive.id;
      const auto & llt = lanelet_map_ptr_->laneletLayer.get(id);
      goal_lanelets_.push_back(llt);
    }
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

std::vector<lanelet::ConstLanelet> RouteHandler::getLanesAfterGoal(
  const double vehicle_length) const
{
  lanelet::ConstLanelet goal_lanelet;
  if (getGoalLanelet(&goal_lanelet)) {
    const double min_succeeding_length = vehicle_length * 2;
    const auto succeeding_lanes_vec = lanelet::utils::query::getSucceedingLaneletSequences(
      routing_graph_ptr_, goal_lanelet, min_succeeding_length);
    if (succeeding_lanes_vec.empty()) {
      return std::vector<lanelet::ConstLanelet>{};
    } else {
      return succeeding_lanes_vec.front();
    }
  } else {
    return std::vector<lanelet::ConstLanelet>{};
  }
}

lanelet::ConstLanelets RouteHandler::getRouteLanelets() const { return route_lanelets_; }

Pose RouteHandler::getGoalPose() const { return route_msg_.goal_pose; }

void RouteHandler::setPullOverGoalPose(
  const lanelet::ConstLanelet target_lane, const double vehicle_width, const double margin)
{
  const auto arc_position_goal =
    lanelet::utils::getArcCoordinates({target_lane}, route_msg_.goal_pose);
  Path centerline_path = convertToPathFromPathWithLaneId(
    getCenterLinePath({target_lane}, 0.0, arc_position_goal.length + 10));
  const auto seg_idx = tier4_autoware_utils::findNearestSegmentIndex(
    centerline_path.points, route_msg_.goal_pose.position);
  const double d_lat = tier4_autoware_utils::calcLongitudinalOffsetToSegment(
    centerline_path.points, seg_idx, route_msg_.goal_pose.position);
  const auto shoulder_point =
    tier4_autoware_utils::calcOffsetPose(centerline_path.points.at(seg_idx).pose, d_lat, 0.0, 0.0);
  pull_over_goal_pose_.orientation = shoulder_point.orientation;
  pull_over_goal_pose_.position = shoulder_point.position;

  // distance between shoulder lane's left boundary and shoulder lane center
  double distance_shoulder_to_left_boundary =
    getDistanceToShoulderBoundary({target_lane}, shoulder_point);

  // distance between shoulder lane center and target line
  double distance_shoulder_to_target =
    distance_shoulder_to_left_boundary + vehicle_width / 2 + margin;

  // Apply shifting shoulder lane to adjust to target line
  double offset = -distance_shoulder_to_target;

  double yaw = tf2::getYaw(shoulder_point.orientation);
  pull_over_goal_pose_.position.x = shoulder_point.position.x - std::sin(yaw) * offset;
  pull_over_goal_pose_.position.y = shoulder_point.position.y + std::cos(yaw) * offset;
}

Pose RouteHandler::getPullOverGoalPose() const { return pull_over_goal_pose_; }

lanelet::Id RouteHandler::getGoalLaneId() const
{
  if (route_msg_.segments.empty()) {
    return lanelet::InvalId;
  } else {
    return route_msg_.segments.back().preferred_primitive_id;
  }
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
  } else {
    return exists(route_msg_.segments.back().primitives, lanelet.id());
  }
}

lanelet::ConstLanelets RouteHandler::getLaneletsFromIds(const lanelet::Ids ids) const
{
  lanelet::ConstLanelets lanelets;
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
  if (getNextLaneletWithinRoute(lanelet, &next_lanelet)) {
    return false;
  }
  return true;
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
    length += boost::geometry::length(next_lanelet.centerline().basicLineString());
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
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRoute(current_lanelet, &prev_lanelet)) {
      break;
    }
    lanelet_sequence_backward.push_back(prev_lanelet);
    length += boost::geometry::length(prev_lanelet.centerline().basicLineString());
    current_lanelet = prev_lanelet;
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
  lanelet::ConstLanelets lanelet_sequence_backward;
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet_sequence_forward = getLaneletSequenceAfter(lanelet, forward_distance);

  const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
  if (arc_coordinate.length < backward_distance) {
    lanelet_sequence_backward = getLaneletSequenceUpTo(lanelet, backward_distance);
  }

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
  lanelet::ConstLanelets lanelet_sequence_backward;
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(route_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet_sequence_forward = getLaneletSequenceAfter(lanelet, forward_distance);

  const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
  if (arc_coordinate.length < backward_distance) {
    lanelet_sequence_backward = getLaneletSequenceUpTo(lanelet, backward_distance);
  }

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

  lanelet::ArcCoordinates arc_coordinates;
  const auto & centerline_2d = to2D(lanelet.centerline());

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
    length += boost::geometry::length(next_lanelet.centerline().basicLineString());
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

  lanelet::ArcCoordinates arc_coordinates;
  const auto & centerline_2d = to2D(lanelet.centerline());

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
    length += boost::geometry::length(prev_lanelet.centerline().basicLineString());
  }

  return lanelet_sequence_backward;
}

lanelet::ConstLanelets RouteHandler::getShoulderLaneletSequence(
  const lanelet::ConstLanelet & lanelet, const Pose & current_pose, const double backward_distance,
  const double forward_distance) const
{
  lanelet::ConstLanelets lanelet_sequence;
  lanelet::ConstLanelets lanelet_sequence_backward;
  lanelet::ConstLanelets lanelet_sequence_forward;
  if (!exists(shoulder_lanelets_, lanelet)) {
    return lanelet_sequence;
  }

  lanelet_sequence_forward = getShoulderLaneletSequenceAfter(lanelet, forward_distance);

  const auto arc_coordinate = lanelet::utils::getArcCoordinates({lanelet}, current_pose);
  if (arc_coordinate.length < backward_distance) {
    lanelet_sequence_backward = getShoulderLaneletSequenceUpTo(lanelet, backward_distance);
  }

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
  lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(lanelet);
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

bool RouteHandler::getPreviousLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * prev_lanelet) const
{
  if (exists(start_lanelets_, lanelet)) {
    return false;
  }
  lanelet::ConstLanelets previous_lanelets = routing_graph_ptr_->previous(lanelet);
  for (const auto & llt : previous_lanelets) {
    if (exists(route_lanelets_, llt)) {
      *prev_lanelet = llt;
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets RouteHandler::getLaneletsFromPoint(const lanelet::ConstPoint3d & point) const
{
  return lanelet::utils::findUsagesInLanelets(*lanelet_map_ptr_, point);
}

bool RouteHandler::getRightLaneletWithinRoute(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * right_lanelet) const
{
  auto opt_right_lanelet = routing_graph_ptr_->right(lanelet);
  if (!!opt_right_lanelet) {
    *right_lanelet = opt_right_lanelet.get();
    return exists(route_lanelets_, *right_lanelet);
  } else {
    return false;
  }
}

bool RouteHandler::getNextLaneletWithinRouteExceptStart(
  const lanelet::ConstLanelet & lanelet, lanelet::ConstLanelet * next_lanelet) const
{
  if (exists(goal_lanelets_, lanelet)) {
    return false;
  }
  lanelet::ConstLanelets following_lanelets = routing_graph_ptr_->following(lanelet);
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
  lanelet::ConstLanelets previous_lanelets = routing_graph_ptr_->previous(lanelet);
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
  auto opt_left_lanelet = routing_graph_ptr_->left(lanelet);
  if (!!opt_left_lanelet) {
    *left_lanelet = opt_left_lanelet.get();
    return exists(route_lanelets_, *left_lanelet);
  } else {
    return false;
  }
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
  return lanelet_map_ptr_->laneletLayer.findUsages(lanelet.rightBound().invert());
}

lanelet::Lanelets RouteHandler::getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet) const
{
  return lanelet_map_ptr_->laneletLayer.findUsages(lanelet.leftBound().invert());
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
  } else if (same) {
    return getRightMostLinestring(same.get());
  } else if (!opposite.empty()) {
    return getLeftMostLinestring(lanelet::ConstLanelet(opposite.front()));
  }
  return {};
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
  } else if (same) {
    return getLeftMostLinestring(same.get());
  } else if (!opposite.empty()) {
    return getRightMostLinestring(lanelet::ConstLanelet(opposite.front()));
  }
  return {};
}

bool RouteHandler::getLaneChangeTarget(
  const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const
{
  for (auto it = lanelets.begin(); it != lanelets.end(); ++it) {
    const auto lanelet = *it;

    int num = getNumLaneToPreferredLane(lanelet);
    if (num == 0) {
      continue;
    }

    if (num < 0) {
      if (!!routing_graph_ptr_->right(lanelet)) {
        auto right_lanelet = routing_graph_ptr_->right(lanelet);
        *target_lanelet = right_lanelet.get();
        return true;
      } else {
        continue;
      }
    }

    if (num > 0) {
      if (!!routing_graph_ptr_->left(lanelet)) {
        auto left_lanelet = routing_graph_ptr_->left(lanelet);
        *target_lanelet = left_lanelet.get();
        return true;
      } else {
        continue;
      }
    }
  }

  *target_lanelet = lanelets.front();
  return false;
}

bool RouteHandler::getPullOverTarget(
  const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet) const
{
  for (const auto & shoulder_lanelet : lanelets) {
    if (lanelet::utils::isInLanelet(getGoalPose(), shoulder_lanelet, 0.1)) {
      *target_lanelet = shoulder_lanelet;
      return true;
    }
  }
  return false;
}

bool RouteHandler::getPullOutStart(
  const lanelet::ConstLanelets & lanelets, lanelet::ConstLanelet * target_lanelet,
  const Pose & pose, const double vehicle_width) const
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
  lanelet::ConstLanelets empty_lanelets;
  if (!getClosestLaneletWithinRoute(pose, &lanelet)) {
    return empty_lanelets;
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
    lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    const lanelet::ConstLineString3d centerline = llt.centerline();

    const auto addPathPoint = [&reference_path, &limit, &llt](const auto & pt) {
      PathPointWithLaneId p{};
      p.point.pose.position = lanelet::utils::conversion::toGeomMsgPt(pt);
      p.lane_ids.push_back(llt.id());
      p.point.longitudinal_velocity_mps = limit.speedLimit.value();
      reference_path.points.push_back(p);
    };

    for (size_t i = 0; i < centerline.size(); i++) {
      const lanelet::ConstPoint3d pt = centerline[i];
      lanelet::ConstPoint3d next_pt =
        (i + 1 < centerline.size()) ? centerline[i + 1] : centerline[i];
      double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

      if (s < s_start && s + distance > s_start) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_start) : pt;
        addPathPoint(p);
      }
      if (s >= s_start && s <= s_end) {
        addPathPoint(pt);
      }
      if (s < s_end && s + distance > s_end) {
        const auto p = use_exact ? get3DPointFrom2DArcLength(lanelet_sequence, s_end) : next_pt;
        addPathPoint(p);
      }
      s += distance;
    }
  }

  reference_path = removeOverlappingPoints(reference_path);

  // append a point only when having one point so that yaw calculation would work
  if (reference_path.points.size() == 1) {
    const int lane_id = reference_path.points.front().lane_ids.front();
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
    lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(llt);
    point.point.longitudinal_velocity_mps = limit.speedLimit.value();
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

  int num = getNumLaneToPreferredLane(lanelet);
  if (num < 0) {
    auto right_lanelet = (!!routing_graph_ptr_->right(lanelet))
                           ? routing_graph_ptr_->right(lanelet)
                           : routing_graph_ptr_->adjacentRight(lanelet);
    target_lanelets = getLaneletSequence(right_lanelet.get());
  }
  if (num > 0) {
    auto left_lanelet = (!!routing_graph_ptr_->left(lanelet))
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

lanelet::routing::RoutingGraphContainer RouteHandler::getOverallGraph() const
{
  return *overall_graphs_ptr_;
}

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

  auto first_lane = lanelet_sequence.front();
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
  lanelet::ConstLanelets lane_sequence_up_to = getLaneSequenceUpTo(lanelet);
  lanelet::ConstLanelets lane_sequence_after = getLaneSequenceAfter(lanelet);

  lane_sequence.insert(lane_sequence.end(), lane_sequence_up_to.begin(), lane_sequence_up_to.end());
  lane_sequence.insert(lane_sequence.end(), lane_sequence_after.begin(), lane_sequence_after.end());
  return lane_sequence;
}

lanelet::ConstLanelets RouteHandler::getLaneSequenceUpTo(
  const lanelet::ConstLanelet & lanelet) const
{
  lanelet::ConstLanelets lane_sequence_backward;
  if (!exists(route_lanelets_, lanelet)) {
    return lane_sequence_backward;
  }

  lanelet::ConstLanelet current_lanelet = lanelet;
  while (rclcpp::ok()) {
    lanelet::ConstLanelet prev_lanelet;
    if (!getPreviousLaneletWithinRoute(current_lanelet, &prev_lanelet)) {
      break;
    }

    lanelet::ConstLanelets current_lanelet_section = getNeighborsWithinRoute(current_lanelet);
    lanelet::ConstLanelets prev_lanelet_section = getNeighborsWithinRoute(prev_lanelet);
    if (!isBijectiveConnection(prev_lanelet_section, current_lanelet_section)) {
      break;
    }
    lane_sequence_backward.push_back(prev_lanelet);
    current_lanelet = prev_lanelet;
  }

  std::reverse(lane_sequence_backward.begin(), lane_sequence_backward.end());
  return lane_sequence_backward;
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

    lanelet::ConstLanelets current_lanelet_section = getNeighborsWithinRoute(current_lanelet);
    lanelet::ConstLanelets next_lanelet_section = getNeighborsWithinRoute(next_lanelet);
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
  lanelet::ConstLanelets neighbor_lanelets =
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
  lanelet::ConstLanelets neighbors = getNeighborsWithinRoute(lanelet);
  std::vector<lanelet::ConstLanelets> lane_section;
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
  lanelet::ConstLanelet final_lanelet = lane_sequence.back();
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
  lanelet::Optional<lanelet::routing::Route> optional_route =
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
  for (const auto & llt : shortest_path) {
    path_lanelets->push_back(llt);
  }
  return true;
}

std::vector<HADMapSegment> RouteHandler::createMapSegments(
  const lanelet::ConstLanelets & path_lanelets) const
{
  const auto main_path = getMainLanelets(path_lanelets);

  std::vector<HADMapSegment> route_sections;

  if (main_path.empty()) {
    return route_sections;
  }

  for (const auto & main_llt : main_path) {
    HADMapSegment route_section_msg;
    lanelet::ConstLanelets route_section_lanelets = getNeighborsWithinRoute(main_llt);
    route_section_msg.preferred_primitive_id = main_llt.id();
    for (const auto & section_llt : route_section_lanelets) {
      MapPrimitive p;
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
