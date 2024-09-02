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

#ifndef AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__MANAGER_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"
#include "autoware/behavior_path_sampling_planner_module/sampling_planner_module.hpp"
#include "autoware/behavior_path_sampling_planner_module/sampling_planner_parameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
class SamplingPlannerModuleManager : public SceneModuleManagerInterface
{
public:
  SamplingPlannerModuleManager() : SceneModuleManagerInterface{"sampling_planner"} {}
  void init(rclcpp::Node * node) override;

  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override
  {
    return std::make_unique<SamplingPlannerModule>(
      name_, *node_, parameters_, rtc_interface_ptr_map_,
      objects_of_interest_marker_interface_ptr_map_, steering_factor_interface_ptr_);
  }

  void updateModuleParams(
    [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters) override;

private:
  std::shared_ptr<SamplingPlannerParameters> parameters_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__MANAGER_HPP_
