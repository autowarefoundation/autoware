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

#include "behavior_path_planner/turn_signal_decider.hpp"

#include "behavior_path_planner/utilities.hpp"

#include <autoware_utils/autoware_utils.hpp>

#include <limits>
#include <string>
#include <utility>

namespace behavior_path_planner
{
TurnIndicatorsCommand TurnSignalDecider::getTurnSignal(
  const PathWithLaneId & path, const Pose & current_pose, const RouteHandler & route_handler,
  const TurnIndicatorsCommand & turn_signal_plan, const double plan_distance) const
{
  auto turn_signal = turn_signal_plan;

  // If the distance to intersection is nearer than path change point,
  // use turn signal for turning at the intersection
  const auto intersection_result = getIntersectionTurnSignal(path, current_pose, route_handler);
  const auto intersection_turn_signal = intersection_result.first;
  const auto intersection_distance = intersection_result.second;

  if (intersection_distance < plan_distance) {
    turn_signal.command = intersection_turn_signal.command;
  }

  return turn_signal;
}

std::pair<TurnIndicatorsCommand, double> TurnSignalDecider::getIntersectionTurnSignal(
  const PathWithLaneId & path, const Pose & current_pose, const RouteHandler & route_handler) const
{
  TurnIndicatorsCommand turn_signal{};
  turn_signal.command = TurnIndicatorsCommand::DISABLE;
  double distance = std::numeric_limits<double>::max();
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};

  if (path.points.empty()) {
    return std::make_pair(turn_signal, distance);
  }

  // Get frenet coordinate of current_pose on path
  util::FrenetCoordinate3d vehicle_pose_frenet;
  if (!util::convertToFrenetCoordinate3d(path, current_pose.position, &vehicle_pose_frenet)) {
    RCLCPP_ERROR_THROTTLE(
      logger_, clock, 5000, "failed to convert vehicle pose into frenet coordinate");
    return std::make_pair(turn_signal, distance);
  }

  // Get nearest intersection and decide turn signal
  double accumulated_distance = 0;

  auto prev_point = path.points.front();
  auto prev_lane_id = lanelet::InvalId;
  for (const auto & path_point : path.points) {
    accumulated_distance += autoware_utils::calcDistance3d(prev_point.point, path_point.point);
    prev_point = path_point;
    const double distance_from_vehicle_front =
      accumulated_distance - vehicle_pose_frenet.length - base_link2front_;
    if (distance_from_vehicle_front < 0.0) {
      continue;
    }
    // TODO(Horibe): Route Handler should be a library.
    for (const auto & lane : route_handler.getLaneletsFromIds(path_point.lane_ids)) {
      if (lane.id() == prev_lane_id) {
        continue;
      }
      prev_lane_id = lane.id();

      if (
        lane.attributeOr("turn_signal_distance", std::numeric_limits<double>::max()) <
        distance_from_vehicle_front) {
        if (1 < path_point.lane_ids.size() && lane.id() == path_point.lane_ids.back()) {
          continue;
        }
      }
      if (lane.attributeOr("turn_direction", std::string("none")) == "left") {
        turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
        distance = distance_from_vehicle_front;
        return std::make_pair(turn_signal, distance);
      }
      if (lane.attributeOr("turn_direction", std::string("none")) == "right") {
        turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
        distance = distance_from_vehicle_front;
        return std::make_pair(turn_signal, distance);
      }
    }
    if (distance_from_vehicle_front > intersection_search_distance_) {
      return std::make_pair(turn_signal, distance);
    }
  }
  return std::make_pair(turn_signal, distance);
}
}  // namespace behavior_path_planner
