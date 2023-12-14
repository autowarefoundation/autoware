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

#ifndef BEHAVIOR_PATH_LANE_CHANGE_MODULE__MANAGER_HPP_
#define BEHAVIOR_PATH_LANE_CHANGE_MODULE__MANAGER_HPP_

#include "behavior_path_lane_change_module/utils/data_structs.hpp"
#include "behavior_path_planner_common/interface/scene_module_manager_interface.hpp"
#include "route_handler/route_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
using route_handler::Direction;

class LaneChangeModuleManager : public SceneModuleManagerInterface
{
public:
  LaneChangeModuleManager(
    const std::string & name, const Direction direction, const LaneChangeModuleType type)
  : SceneModuleManagerInterface{name}, direction_{direction}, type_{type}
  {
  }

  void init(rclcpp::Node * node) override;

  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override;

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) override;

protected:
  std::shared_ptr<LaneChangeParameters> parameters_;

  Direction direction_;

  LaneChangeModuleType type_;
};

class LaneChangeRightModuleManager : public LaneChangeModuleManager
{
public:
  LaneChangeRightModuleManager()
  : LaneChangeModuleManager(
      "lane_change_right", route_handler::Direction::RIGHT, LaneChangeModuleType::NORMAL)
  {
  }
};

class LaneChangeLeftModuleManager : public LaneChangeModuleManager
{
public:
  LaneChangeLeftModuleManager()
  : LaneChangeModuleManager(
      "lane_change_left", route_handler::Direction::LEFT, LaneChangeModuleType::NORMAL)
  {
  }
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_LANE_CHANGE_MODULE__MANAGER_HPP_
