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

#ifndef BEHAVIOR_PATH_PLANNER__TURN_SIGNAL_DECIDER_HPP_
#define BEHAVIOR_PATH_PLANNER__TURN_SIGNAL_DECIDER_HPP_

#include <route_handler/route_handler.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <utility>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::Pose;
using route_handler::RouteHandler;
class TurnSignalDecider
{
public:
  TurnIndicatorsCommand getTurnSignal(
    const PathWithLaneId & path, const Pose & current_pose, const RouteHandler & route_handler,
    const TurnIndicatorsCommand & turn_signal_plan, const double plan_distance) const;

  void setParameters(const double base_link2front, const double intersection_search_distance)
  {
    base_link2front_ = base_link2front;
    intersection_search_distance_ = intersection_search_distance;
  }

private:
  std::pair<TurnIndicatorsCommand, double> getIntersectionTurnSignal(
    const PathWithLaneId & path, const Pose & current_pose,
    const RouteHandler & route_handler) const;

  rclcpp::Logger logger_{
    rclcpp::get_logger("behavior_path_planner").get_child("turn_signal_decider")};

  // data
  double intersection_search_distance_{0.0};
  double base_link2front_{0.0};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__TURN_SIGNAL_DECIDER_HPP_
