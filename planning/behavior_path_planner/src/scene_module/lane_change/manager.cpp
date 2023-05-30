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
using utils::convertToSnakeCase;
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

void LaneChangeModuleManager::updateModuleParams(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto p = parameters_;

  const std::string ns = name_ + ".";
  updateParam<double>(
    parameters, ns + "finish_judge_lateral_threshold", p->finish_judge_lateral_threshold);

  std::for_each(registered_modules_.begin(), registered_modules_.end(), [&p](const auto & m) {
    m->updateModuleParams(p);
  });
}

AvoidanceByLaneChangeModuleManager::AvoidanceByLaneChangeModuleManager(
  rclcpp::Node * node, const std::string & name, const ModuleConfigParameters & config,
  std::shared_ptr<LaneChangeParameters> parameters,
  std::shared_ptr<AvoidanceParameters> avoidance_parameters,
  std::shared_ptr<AvoidanceByLCParameters> avoidance_by_lane_change_parameters)
: LaneChangeModuleManager(
    node, name, config, std::move(parameters), Direction::NONE,
    LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE),
  avoidance_parameters_(std::move(avoidance_parameters)),
  avoidance_by_lane_change_parameters_(std::move(avoidance_by_lane_change_parameters))
{
  rtc_interface_ptr_map_.clear();
  const std::vector<std::string> rtc_types = {"left", "right"};
  for (const auto & rtc_type : rtc_types) {
    const auto snake_case_name = convertToSnakeCase(name);
    const std::string rtc_interface_name = snake_case_name + "_" + rtc_type;
    rtc_interface_ptr_map_.emplace(
      rtc_type, std::make_shared<RTCInterface>(node, rtc_interface_name));
  }
}

std::shared_ptr<SceneModuleInterface>
AvoidanceByLaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_shared<AvoidanceByLaneChangeInterface>(
    name_, *node_, parameters_, avoidance_parameters_, avoidance_by_lane_change_parameters_,
    rtc_interface_ptr_map_);
}

}  // namespace behavior_path_planner
