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

#include "object_stop_decision.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <limits>

namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
{
void update_object_map(
  ObjectStopDecisionMap & object_map, const std::vector<Collision> & collisions,
  const rclcpp::Time & now, const TrajectoryPoints & trajectory, const PlannerParam & params)
{
  for (auto & [object, decision] : object_map) decision.collision_detected = false;
  for (const auto & collision : collisions) {
    if (auto search = object_map.find(collision.object_uuid); search != object_map.end()) {
      search->second.collision_detected = true;
      const auto is_closer_collision_point =
        autoware::motion_utils::calcSignedArcLength(
          trajectory, search->second.collision_point, collision.point) < 0.0;
      if (is_closer_collision_point) search->second.collision_point = collision.point;
    } else {
      object_map[collision.object_uuid].collision_point = collision.point;
    }
  }
  for (auto it = object_map.begin(); it != object_map.end();) {
    auto & decision = it->second;
    decision.update_timers(now, params.add_duration_buffer, params.remove_duration_buffer);
    if (decision.is_inactive())
      it = object_map.erase(it);
    else
      ++it;
  }
}

std::optional<geometry_msgs::msg::Point> find_earliest_collision(
  const ObjectStopDecisionMap & object_map, const EgoData & ego_data)
{
  std::optional<geometry_msgs::msg::Point> earliest_collision;
  double earliest_collision_arc_length = std::numeric_limits<double>::max();
  for (auto & [object_uuid, decision] : object_map) {
    if (decision.should_be_avoided()) {
      const auto arc_length = autoware::motion_utils::calcSignedArcLength(
        ego_data.trajectory, ego_data.pose.position, decision.collision_point);
      if (arc_length < earliest_collision_arc_length) {
        earliest_collision_arc_length = arc_length;
        earliest_collision = decision.collision_point;
      }
    }
  }
  return earliest_collision;
}

}  // namespace autoware::motion_velocity_planner::dynamic_obstacle_stop
