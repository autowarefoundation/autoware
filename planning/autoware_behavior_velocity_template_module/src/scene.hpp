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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using ::behavior_velocity_planner::SceneModuleInterface;
using ::behavior_velocity_planner::StopReason;

class TemplateModule : public SceneModuleInterface
{
public:
  TemplateModule(
    const int64_t module_id, const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Modify the velocity of path points.
   *
   * This method is responsible for adjusting the velocity of each point in the input path based on
   * specific criteria.
   *
   * @param path A pointer to the path containing points to be modified.
   * @param stop_reason A pointer to the stop reason data.
   * @return [bool] wether the path velocity was modified or not.
   */
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  /**
   * @brief Create a visualization of debug markers.
   *
   * This method is responsible for generating a visualization of debug markers and returning them
   * as a `visualization_msgs::msg::MarkerArray`.
   *
   * @return A `visualization_msgs::msg::MarkerArray` containing debug markers.
   */
  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

  /**
   * @brief Create virtual walls for the scene.
   *
   * This method is responsible for generating virtual walls for the scene and returning them as a
   * `motion_utils::VirtualWalls` object.
   *
   * @return A `motion_utils::VirtualWalls` object representing virtual walls in the scene.
   */
  motion_utils::VirtualWalls createVirtualWalls() override;
};

}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
