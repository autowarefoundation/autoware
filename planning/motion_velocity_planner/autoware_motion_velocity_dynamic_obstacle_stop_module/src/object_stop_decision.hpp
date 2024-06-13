// Copyright 2024 TIER IV, Inc.
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

#ifndef OBJECT_STOP_DECISION_HPP_
#define OBJECT_STOP_DECISION_HPP_

#include "types.hpp"

#include <rclcpp/time.hpp>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
/// @brief representation of a decision to stop for a dynamic object
struct ObjectStopDecision
{
  geometry_msgs::msg::Point collision_point{};
  bool collision_detected = true;
  std::optional<rclcpp::Time> start_detection_time{};
  std::optional<rclcpp::Time> last_stop_decision_time{};

public:
  void update_timers(const rclcpp::Time & now, const double add_buffer, const double remove_buffer)
  {
    if (collision_detected) {
      if (!start_detection_time) start_detection_time = now;
      if ((now - *start_detection_time).seconds() >= add_buffer) last_stop_decision_time = now;
    } else if (
      !last_stop_decision_time || (now - *last_stop_decision_time).seconds() >= remove_buffer) {
      start_detection_time.reset();
    }
  }
  bool should_be_avoided() const { return start_detection_time && last_stop_decision_time; }
  bool is_inactive() const { return !start_detection_time; }
};
using ObjectStopDecisionMap = std::unordered_map<std::string, ObjectStopDecision>;

/// @brief update an object map with the given detected collisions
/// @details update the collision point of each object in the map (keep the closest one) and its
/// timers, remove inactive objects and add new ones
/// @param [inout] object_map object map to update
/// @param [in] collisions detected collisions (used to update the objects' collision points)
/// @param [in] now current time (used to update the objects' timers)
/// @param [in] trajectory ego trajectory points (used to determine the closest collision points)
/// @param [in] params planner parameters
void update_object_map(
  ObjectStopDecisionMap & object_map, const std::vector<Collision> & collisions,
  const rclcpp::Time & now, const TrajectoryPoints & trajectory, const PlannerParam & params);

/// @brief find the earliest collision requiring a stop along the ego trajectory
/// @param object_map map with the objects to avoid and their corresponding collision points
/// @param ego_data ego data
/// @return the earliest collision point (if any)
std::optional<geometry_msgs::msg::Point> find_earliest_collision(
  const ObjectStopDecisionMap & object_map, const EgoData & ego_data);

}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop

#endif  // OBJECT_STOP_DECISION_HPP_
