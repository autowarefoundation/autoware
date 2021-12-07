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

#include "behavior_path_planner/scene_module/scene_module_bt_node_interface.hpp"

#include <memory>
#include <string>

namespace behavior_path_planner
{
BT::NodeStatus isExecutionRequested(
  const std::shared_ptr<const SceneModuleInterface> p,
  const std::shared_ptr<SceneModuleStatus> & status)
{
  const auto ret = p->isExecutionRequested();
  status->is_requested = ret;
  RCLCPP_DEBUG_STREAM(p->getLogger(), "name = " << p->name() << ", result = " << ret);
  return ret ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus isExecutionReady(
  const std::shared_ptr<const SceneModuleInterface> p,
  const std::shared_ptr<SceneModuleStatus> & status)
{
  const auto ret = p->isExecutionReady();
  status->is_ready = ret;
  RCLCPP_DEBUG_STREAM(p->getLogger(), "name = " << p->name() << ", result = " << ret);
  return ret ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

SceneModuleBTNodeInterface::SceneModuleBTNodeInterface(
  const std::string & name, const BT::NodeConfiguration & config,
  const std::shared_ptr<SceneModuleInterface> & scene_module,
  const std::shared_ptr<SceneModuleStatus> & module_status)
: BT::CoroActionNode(name, config), scene_module_(scene_module), module_status_(module_status)
{
}

BT::NodeStatus SceneModuleBTNodeInterface::tick()
{
  RCLCPP_DEBUG_STREAM(
    scene_module_->getLogger(), "bt::tick is called. module name: " << scene_module_->name());
  auto current_status = BT::NodeStatus::RUNNING;

  scene_module_->onEntry();
  scene_module_->updateApproval();

  const bool is_waiting_approval = scene_module_->approval_handler_.isWaitingApproval() &&
                                   !scene_module_->approval_handler_.isApproved();
  if (is_waiting_approval) {
    try {
      // NOTE: Since BehaviorTreeCpp has an issue to shadow the exception reason thrown
      // in the TreeNode, catch and display it here until the issue is fixed.
      scene_module_->updateData();
      auto res = setOutput<BehaviorModuleOutput>("output", scene_module_->planWaitingApproval());
      if (!res) {
        RCLCPP_ERROR_STREAM(scene_module_->getLogger(), "setOutput() failed : " << res.error());
      }
      module_status_->is_waiting_approval = scene_module_->approval_handler_.isWaitingApproval();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        scene_module_->getLogger(), "behavior module has failed with exception: " << e.what());
      // std::exit(EXIT_FAILURE);  // TODO(Horibe) do appropriate handing
    }
    return BT::NodeStatus::SUCCESS;
  }

  while (rclcpp::ok()) {
    // NOTE: Since BehaviorTreeCpp has an issue to shadow the exception reason thrown
    // in the TreeNode, catch and display it here until the issue is fixed.
    try {
      auto res = setOutput<BehaviorModuleOutput>("output", scene_module_->run());
      if (!res) {
        RCLCPP_ERROR_STREAM(scene_module_->getLogger(), "setOutput() failed : " << res.error());
      }

      current_status = scene_module_->updateState();

      // for data output
      module_status_->status = current_status;
      module_status_->is_waiting_approval = scene_module_->approval_handler_.isWaitingApproval();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        scene_module_->getLogger(), "behavior module has failed with exception: " << e.what());
      // std::exit(EXIT_FAILURE);  // TODO(Horibe) do appropriate handing
    }

    RCLCPP_DEBUG_STREAM(
      scene_module_->getLogger(), "on tick: current status = " << BT::toStr(current_status));
    if (current_status != BT::NodeStatus::RUNNING) {
      RCLCPP_DEBUG(scene_module_->getLogger(), "on tick: module ended.");
      break;
    }

    setStatusRunningAndYield();
  }

  scene_module_->onExit();
  RCLCPP_DEBUG_STREAM(
    scene_module_->getLogger(), "on tick: return current status = " << BT::toStr(current_status));

  return current_status;
}

void SceneModuleBTNodeInterface::halt()
{
  scene_module_->onExit();
  BT::CoroActionNode::halt();
}

BT::NodeStatus SceneModuleBTNodeInterface::planCandidate(BT::TreeNode & self)
{
  RCLCPP_DEBUG_STREAM(
    scene_module_->getLogger(), "bt::planCandidate module name: " << scene_module_->name());
  auto res = self.setOutput<BehaviorModuleOutput>("output", scene_module_->planWaitingApproval());
  if (!res) {
    RCLCPP_ERROR_STREAM(scene_module_->getLogger(), "setOutput() failed : " << res.error());
  }

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SceneModuleBTNodeInterface::providedPorts()
{
  return {BT::OutputPort<BehaviorModuleOutput>("output", "desc")};
}

}  // namespace behavior_path_planner
