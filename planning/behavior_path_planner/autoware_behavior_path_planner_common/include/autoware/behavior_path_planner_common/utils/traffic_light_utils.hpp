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
#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__TRAFFIC_LIGHT_UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__TRAFFIC_LIGHT_UTILS_HPP_

#include <autoware/behavior_path_planner_common/data_manager.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <limits>
#include <memory>

namespace autoware::behavior_path_planner::utils::traffic_light
{

using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;
using geometry_msgs::msg::Pose;

/**
 * @brief Computes the distance from the current position to the next traffic light along a set of
 * lanelets.
 *
 * This function finds the closest lanelet to the current position and then searches for the
 * next traffic light within the lanelet sequence. If a traffic light is found, it calculates
 * the distance to the stop line of that traffic light from the current position.
 *
 * @param current_pose The current position of ego vehicle.
 * @param lanelets A collection of lanelets representing the road ahead.
 * @return The distance to the next traffic light's stop line or infinity if no traffic light is
 * found ahead.
 */
double getDistanceToNextTrafficLight(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets);

/**
 * @brief Calculates the signed distance from the ego vehicle to the stop line of the nearest red
 * traffic light.
 *
 * Iterates over lanelets to find traffic lights. If a red traffic light is found,
 * computes the distance to its stop line from the ego vehicle's position along a specified path.
 *
 * @param lanelets Collection of lanelets to inspect for traffic light regulatory elements.
 * @param path The path along which the distance from ego current position to the stop line is
 * measured.
 * @param planner_data Shared pointer to planner data with vehicle odometry and traffic signal
 * information.
 * @return Optional double value with the signed distance to the stop line, or std::nullopt if no
 * red traffic light is detected.
 */
std::optional<double> calcDistanceToRedTrafficLight(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data);

/**
 * @brief Checks if the vehicle is stationary within a specified distance from a red traffic light.
 *
 * This function first checks if the vehicle's velocity is below a minimum threshold, indicating it
 * is stopped. It then calculates the distance to the nearest red traffic light using the
 * calcDistanceToRedTrafficLight function. If the vehicle is within the specified distance threshold
 * from the red traffic light, the function returns true, otherwise false.
 *
 * @param lanelets The lanelets to search for traffic lights.
 * @param path The path along which to measure the distance to the traffic light.
 * @param planner_data Shared pointer to the planner data containing vehicle state information.
 * @param distance_threshold The maximum allowable distance from a red traffic light to consider the
 * vehicle stopped.
 * @return True if the vehicle is stopped within the distance threshold from a red traffic light,
 * false otherwise.
 */
bool isStoppedAtRedTrafficLightWithinDistance(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data,
  const double distance_threshold = std::numeric_limits<double>::infinity());

/**
 * @brief Determines if a traffic signal indicates a stop for the given lanelet.
 *
 * Evaluates the current state of the traffic light, considering if it's green or unknown,
 * which would not necessitate a stop. Then, it checks the turn direction attribute of the lanelet
 * against the traffic light's arrow shapes to determine whether a vehicle must stop or if it can
 * proceed based on allowed turn directions.
 *
 * @param lanelet The lanelet to check for a stop signal at its traffic light.
 * @param planner_data Shared pointer to the planner data containing vehicle state information.
 * @return True if the traffic signal indicates a stop is required, false otherwise.
 */
bool isTrafficSignalStop(
  const lanelet::ConstLanelets & lanelets, const std::shared_ptr<const PlannerData> & planner_data);
}  // namespace autoware::behavior_path_planner::utils::traffic_light

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__TRAFFIC_LIGHT_UTILS_HPP_
