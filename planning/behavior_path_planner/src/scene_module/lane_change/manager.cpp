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

#include "behavior_path_planner/scene_module/lane_change/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using route_handler::Direction;
LaneChangeModuleManager::LaneChangeModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  std::shared_ptr<LaneChangeParameters> parameters, const Direction direction,
  const LaneChangeModuleType type)
: SceneModuleManagerInterface(node, name, config, {""}),
  parameters_{std::move(parameters)},
  direction_{direction},
  type_{type}
{
}

std::shared_ptr<SceneModuleInterface> LaneChangeModuleManager::createNewSceneModuleInstance()
{
  if (type_ == LaneChangeModuleType::NORMAL) {
    return std::make_shared<LaneChangeInterface>(
      name_, *node_, parameters_, rtc_interface_ptr_map_,
      std::make_unique<NormalLaneChange>(parameters_, LaneChangeModuleType::NORMAL, direction_));
  }
  return std::make_shared<LaneChangeInterface>(
    name_, *node_, parameters_, rtc_interface_ptr_map_,
    std::make_unique<ExternalRequestLaneChange>(parameters_, direction_));
}

void LaneChangeModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  [[maybe_unused]] auto p = parameters_;

  [[maybe_unused]] const std::string ns = name_ + ".";
  // updateParam<bool>(parameters, ns + ..., ...);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m->updateModuleParams(p);
  });
}

}  // namespace behavior_path_planner
