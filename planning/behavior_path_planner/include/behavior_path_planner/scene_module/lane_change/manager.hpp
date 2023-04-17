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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__MANAGER_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__MANAGER_HPP_

#include "behavior_path_planner/scene_module/lane_change/interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_manager_interface.hpp"
#include "route_handler/route_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace behavior_path_planner
{
using route_handler::Direction;

class LaneChangeModuleManager : public SceneModuleManagerInterface
{
public:
  LaneChangeModuleManager(
    rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
    std::shared_ptr<LaneChangeParameters> parameters, const Direction direction,
    const LaneChangeModuleType type);

  std::shared_ptr<SceneModuleInterface> createNewSceneModuleInstance() override;

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) override;

private:
  std::shared_ptr<LaneChangeParameters> parameters_;

  std::vector<std::shared_ptr<LaneChangeInterface>> registered_modules_;

  Direction direction_;

  LaneChangeModuleType type_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__MANAGER_HPP_
