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

#ifndef BEHAVIOR_PATH_START_PLANNER_MODULE__MANAGER_HPP_
#define BEHAVIOR_PATH_START_PLANNER_MODULE__MANAGER_HPP_

#include "behavior_path_planner_common/interface/scene_module_manager_interface.hpp"
#include "behavior_path_start_planner_module/start_planner_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

class StartPlannerModuleManager : public SceneModuleManagerInterface
{
public:
  StartPlannerModuleManager() : SceneModuleManagerInterface{"start_planner"} {}

  void init(rclcpp::Node * node) override;

  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override
  {
    return std::make_unique<StartPlannerModule>(
      name_, *node_, parameters_, rtc_interface_ptr_map_,
      objects_of_interest_marker_interface_ptr_map_);
  }

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) override;

  bool isSimultaneousExecutableAsApprovedModule() const override;

  bool isSimultaneousExecutableAsCandidateModule() const override;

private:
  std::shared_ptr<StartPlannerParameters> parameters_;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_START_PLANNER_MODULE__MANAGER_HPP_
