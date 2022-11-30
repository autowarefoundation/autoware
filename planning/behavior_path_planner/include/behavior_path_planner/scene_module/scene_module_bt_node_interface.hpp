// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_BT_NODE_INTERFACE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_BT_NODE_INTERFACE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

#include <behaviortree_cpp_v3/bt_factory.h>

#include <memory>
#include <string>

namespace behavior_path_planner
{
struct SceneModuleStatus
{
  explicit SceneModuleStatus(const std::string & n) : module_name(n) {}
  std::string module_name;  // TODO(Horibe) should be const
  bool is_requested{false};
  bool is_execution_ready{false};
  bool is_waiting_approval{false};
  BT::NodeStatus status{BT::NodeStatus::SUCCESS};
};

class SceneModuleBTNodeInterface : public BT::CoroActionNode
{
public:
  SceneModuleBTNodeInterface(
    const std::string & name, const BT::NodeConfiguration & config,
    const std::shared_ptr<SceneModuleInterface> & scene_module,
    const std::shared_ptr<SceneModuleStatus> & module_status);

protected:
  std::shared_ptr<SceneModuleInterface> scene_module_;
  std::shared_ptr<SceneModuleStatus> module_status_;

public:
  BT::NodeStatus tick() override;
  void halt() override;
  static BT::PortsList providedPorts();

  BT::NodeStatus planCandidate(BT::TreeNode & self);
};

BT::NodeStatus isExecutionRequested(
  const std::shared_ptr<const SceneModuleInterface> p,
  const std::shared_ptr<SceneModuleStatus> & status);

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__SCENE_MODULE_BT_NODE_INTERFACE_HPP_
