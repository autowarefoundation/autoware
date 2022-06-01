// Copyright 2022 TIER IV, Inc.
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

#ifndef SCENE_MODULE__RUN_OUT__MANAGER_HPP_
#define SCENE_MODULE__RUN_OUT__MANAGER_HPP_

#include "scene_module/run_out/scene.hpp"
#include "scene_module/scene_module_interface.hpp"

#include <memory>

namespace behavior_velocity_planner
{
class RunOutModuleManager : public SceneModuleManagerInterface
{
public:
  explicit RunOutModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "run_out"; }

private:
  run_out_utils::PlannerParam planner_param_;
  std::shared_ptr<RunOutDebug> debug_ptr_;
  std::unique_ptr<DynamicObstacleCreator> dynamic_obstacle_creator_;

  void launchNewModules(const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path) override;

  void setDynamicObstacleCreator(rclcpp::Node & node);
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__RUN_OUT__MANAGER_HPP_
