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

#ifndef DECISIONS_HPP_
#define DECISIONS_HPP_

#include "types.hpp"

#include <rclcpp/logger.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::out_of_lane
{
/// @brief calculate the distance along the ego path between ego and some target path index
/// @param [in] ego_data data related to the ego vehicle
/// @param [in] target_idx target ego path index
/// @return distance between ego and the target [m]
double distance_along_path(const EgoData & ego_data, const size_t target_idx);
/// @brief estimate the time when ego will reach some target path index
/// @param [in] ego_data data related to the ego vehicle
/// @param [in] target_idx target ego path index
/// @param [in] min_velocity minimum ego velocity used to estimate the time
/// @return time taken by ego to reach the target [s]
double time_along_path(const EgoData & ego_data, const size_t target_idx);
/// @brief use an object's predicted paths to estimate the times it will reach the enter and exit
/// points of an overlapping range
/// @details times when the predicted paths of the object enters/exits the range are calculated
/// but may not exist (e.g,, predicted path ends before reaching the end of the range)
/// @param [in] object dynamic object
/// @param [in] range overlapping range
/// @param [in] route_handler route handler used to estimate the path of the dynamic object
/// @param [in] logger ros logger
/// @return an optional pair (time at enter [s], time at exit [s]). If the dynamic object drives in
/// the opposite direction, time at enter > time at exit
std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range,
  const std::shared_ptr<route_handler::RouteHandler> route_handler, const double dist_buffer,
  const rclcpp::Logger & logger);
/// @brief use the lanelet map to estimate the times when an object will reach the enter and exit
/// points of an overlapping range
/// @param [in] object dynamic object
/// @param [in] range overlapping range
/// @param [in] inputs information used to take decisions (ranges, ego and objects data, route
/// handler, lanelets)
/// @param [in] dist_buffer extra distance used to estimate if a collision will occur on the range
/// @param [in] logger ros logger
/// @return an optional pair (time at enter [s], time at exit [s]). If the dynamic object drives in
/// the opposite direction, time at enter > time at exit.
std::optional<std::pair<double, double>> object_time_to_range(
  const autoware_auto_perception_msgs::msg::PredictedObject & object, const OverlapRange & range,
  const DecisionInputs & inputs, const rclcpp::Logger & logger);
/// @brief decide whether an object is coming in the range at the same time as ego
/// @details the condition depends on the mode (threshold, intervals, ttc)
/// @param [in] range_times times when ego and the object enter/exit the range
/// @param [in] params parameters
/// @param [in] logger ros logger
bool will_collide_on_range(
  const RangeTimes & range_times, const PlannerParam & params, const rclcpp::Logger & logger);
/// @brief check whether we should avoid entering the given range
/// @param [in] range the range to check
/// @param [in] inputs information used to take decisions (ranges, ego and objects data, route
/// handler, lanelets)
/// @param [in] params parameters
/// @param [in] logger ros logger
/// @return return true if we should avoid entering the range
bool should_not_enter(
  const OverlapRange & range, const DecisionInputs & inputs, const PlannerParam & params,
  const rclcpp::Logger & logger);
/// @brief set the velocity of a decision (or unset it) based on the distance away from the range
/// @param [out] decision decision to update (either set its velocity or unset the decision)
/// @param [in] distance distance between ego and the range corresponding to the decision
/// @param [in] params parameters
void set_decision_velocity(
  std::optional<Slowdown> & decision, const double distance, const PlannerParam & params);
/// @brief calculate the decision to slowdown or stop before an overlapping range
/// @param [in] range the range to check
/// @param [in] inputs information used to take decisions (ranges, ego and objects data, route
/// handler, lanelets)
/// @param [in] params parameters
/// @param [in] logger ros logger
/// @return return an optional decision to slowdown or stop
std::optional<Slowdown> calculate_decision(
  const OverlapRange & range, const DecisionInputs & inputs, const PlannerParam & params,
  const rclcpp::Logger & logger);
/// @brief calculate decisions to slowdown or stop before some overlapping ranges
/// @param [in] inputs information used to take decisions (ranges, ego and objects data, route
/// handler, lanelets)
/// @param [in] params parameters
/// @param [in] logger ros logger
/// @return return the calculated decisions to slowdown or stop
std::vector<Slowdown> calculate_decisions(
  const DecisionInputs & inputs, const PlannerParam & params, const rclcpp::Logger & logger);
}  // namespace behavior_velocity_planner::out_of_lane

#endif  // DECISIONS_HPP_
