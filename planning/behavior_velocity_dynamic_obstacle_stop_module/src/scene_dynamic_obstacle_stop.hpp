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

#ifndef SCENE_DYNAMIC_OBSTACLE_STOP_HPP_
#define SCENE_DYNAMIC_OBSTACLE_STOP_HPP_

#include "types.hpp"

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{
class DynamicObstacleStopModule : public SceneModuleInterface
{
public:
  DynamicObstacleStopModule(
    const int64_t module_id, PlannerParam planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock);

  /// @brief insert stop or slow down points to prevent dangerously entering another lane
  /// @param [inout] path the path to update
  /// @param [inout] stop_reason reason for stopping
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  motion_utils::VirtualWalls createVirtualWalls() override;

private:
  PlannerParam params_;
  rclcpp::Time prev_stop_decision_time_;
  std::optional<geometry_msgs::msg::Pose> current_stop_pose_;

protected:
  int64_t module_id_{};

  // Debug
  mutable DebugData debug_data_;
};
}  // namespace behavior_velocity_planner::dynamic_obstacle_stop

#endif  // SCENE_DYNAMIC_OBSTACLE_STOP_HPP_
