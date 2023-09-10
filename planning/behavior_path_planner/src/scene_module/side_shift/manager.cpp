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

#include "behavior_path_planner/scene_module/side_shift/manager.hpp"

#include "tier4_autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{

SideShiftModuleManager::SideShiftModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config)
: SceneModuleManagerInterface(node, name, config, {})
{
  SideShiftParameters p{};

  p.min_distance_to_start_shifting =
    node->declare_parameter<double>(name + ".min_distance_to_start_shifting");
  p.time_to_start_shifting = node->declare_parameter<double>(name + ".time_to_start_shifting");
  p.shifting_lateral_jerk = node->declare_parameter<double>(name + ".shifting_lateral_jerk");
  p.min_shifting_distance = node->declare_parameter<double>(name + ".min_shifting_distance");
  p.min_shifting_speed = node->declare_parameter<double>(name + ".min_shifting_speed");
  p.shift_request_time_limit = node->declare_parameter<double>(name + ".shift_request_time_limit");
  p.publish_debug_marker = node->declare_parameter<bool>(name + ".publish_debug_marker");

  parameters_ = std::make_shared<SideShiftParameters>(p);
}

void SideShiftModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  [[maybe_unused]] auto p = parameters_;

  [[maybe_unused]] std::string ns = "side_shift.";
  // updateParam<bool>(parameters, ns + ..., ...);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace behavior_path_planner
