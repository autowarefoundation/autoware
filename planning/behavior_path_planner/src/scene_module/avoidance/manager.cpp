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

#include "behavior_path_planner/scene_module/avoidance/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

AvoidanceModuleManager::AvoidanceModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  const std::shared_ptr<AvoidanceParameters> & parameters)
: SceneModuleManagerInterface(node, name, config, {"left", "right"}), parameters_{parameters}
{
}

void AvoidanceModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = parameters_;

  std::string ns = "avoidance.";
  updateParam<bool>(parameters, ns + "enable_safety_check", p->enable_safety_check);
  updateParam<bool>(parameters, ns + "publish_debug_marker", p->publish_debug_marker);
  updateParam<bool>(parameters, ns + "print_debug_info", p->print_debug_info);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m->updateModuleParams(p);
  });
}
}  // namespace behavior_path_planner
