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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "autoware_behavior_path_lane_change_module/manager.hpp"
#include "data_structs.hpp"
#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::LaneChangeModuleManager;
using autoware::behavior_path_planner::LaneChangeModuleType;
using autoware::behavior_path_planner::SceneModuleInterface;

class AvoidanceByLaneChangeModuleManager : public LaneChangeModuleManager
{
public:
  AvoidanceByLaneChangeModuleManager()
  : LaneChangeModuleManager(
      "avoidance_by_lane_change", autoware::route_handler::Direction::NONE,
      LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE)
  {
  }

  void init(rclcpp::Node * node) override;

  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override;

private:
  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters_;
};
}  // namespace autoware::behavior_path_planner

#endif  // MANAGER_HPP_
