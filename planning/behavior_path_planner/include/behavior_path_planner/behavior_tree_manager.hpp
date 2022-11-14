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

#ifndef BEHAVIOR_PATH_PLANNER__BEHAVIOR_TREE_MANAGER_HPP_
#define BEHAVIOR_PATH_PLANNER__BEHAVIOR_TREE_MANAGER_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/scene_module/scene_module_bt_node_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
struct BehaviorTreeManagerParam
{
  std::string bt_tree_config_path;
  int groot_zmq_publisher_port;
  int groot_zmq_server_port;
};

class BehaviorTreeManager
{
public:
  BehaviorTreeManager(rclcpp::Node & node, const BehaviorTreeManagerParam & param);
  void createBehaviorTree();
  void registerSceneModule(const std::shared_ptr<SceneModuleInterface> & p);

  void resetBehaviorTree();

  BehaviorModuleOutput run(const std::shared_ptr<PlannerData> & data);
  std::vector<std::shared_ptr<SceneModuleStatus>> getModulesStatus();
  std::shared_ptr<SceneModuleVisitor> getAllSceneModuleDebugMsgData();

  AvoidanceDebugMsgArray getAvoidanceDebugMsgArray();

private:
  BehaviorTreeManagerParam bt_manager_param_;
  std::shared_ptr<PlannerData> current_planner_data_;
  std::vector<std::shared_ptr<SceneModuleInterface>> scene_modules_;
  std::vector<std::shared_ptr<SceneModuleStatus>> modules_status_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  std::shared_ptr<SceneModuleVisitor> scene_module_visitor_ptr_;

  BT::BehaviorTreeFactory bt_factory_;
  BT::Tree bt_tree_;
  BT::Blackboard::Ptr blackboard_;

  BT::NodeStatus checkForceApproval(const std::string & name);

  // For Groot monitoring
  std::unique_ptr<BT::PublisherZMQ> groot_monitor_;

  void addGrootMonitoring(
    BT::Tree * tree, uint16_t publisher_port, uint16_t server_port,
    uint16_t max_msg_per_second = 25);

  void resetGrootMonitor();
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__BEHAVIOR_TREE_MANAGER_HPP_
