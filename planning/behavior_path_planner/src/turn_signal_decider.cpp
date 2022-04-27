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

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

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
    RCLCPP_DEBUG_THROTTLE(
      logger_, clock, 5000, "failed to convert vehicle pose into frenet coordinate");
    return std::make_pair(turn_signal, distance);
  }

  // Get nearest intersection and decide turn signal
  double accumulated_distance = 0;

  auto prev_point = path.points.front();
  auto lane_attribute = std::string("none");
  for (const auto & path_point : path.points) {
    const double path_point_distance =
      tier4_autoware_utils::calcDistance3d(prev_point.point, path_point.point);
    accumulated_distance += path_point_distance;
    prev_point = path_point;
    const double distance_from_vehicle_front =
      accumulated_distance - vehicle_pose_frenet.length - base_link2front_;
    if (distance_from_vehicle_front > intersection_search_distance_) {
      return std::make_pair(turn_signal, distance);
    }
    // TODO(Horibe): Route Handler should be a library.
    for (const auto & lane : route_handler.getLaneletsFromIds(path_point.lane_ids)) {
      // judgement of lighting of turn_signal
      bool lighting_turn_signal = false;
      if (lane.attributeOr("turn_direction", std::string("none")) != lane_attribute) {
        if (
          distance_from_vehicle_front <
            lane.attributeOr("turn_signal_distance", intersection_search_distance_) &&
          path_point_distance > 0.0) {
          lighting_turn_signal = true;
        }
      } else {
        if (
          lane.hasAttribute("turn_direction") &&
          distance_from_vehicle_front < path_point_distance && distance_from_vehicle_front > 0) {
          lighting_turn_signal = true;
        }
      }
      lane_attribute = lane.attributeOr("turn_direction", std::string("none"));

      if (lighting_turn_signal) {
        if (lane_attribute == std::string("left")) {
          turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
        } else if (lane_attribute == std::string("right")) {
          turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
        }
        distance = distance_from_vehicle_front;
      }
    }
  }
  return std::make_pair(turn_signal, distance);
}
}  // namespace behavior_path_planner
