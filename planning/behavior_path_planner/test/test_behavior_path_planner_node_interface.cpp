// Copyright 2023 Tier IV, Inc.
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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behavior_path_planner/behavior_path_planner_node.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionRoute)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<planning_test_utils::PlanningInterfaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};

  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto behavior_path_planner_dir =
    ament_index_cpp::get_package_share_directory("behavior_path_planner");

  node_options.append_parameter_override(
    "bt_tree_config_path", behavior_path_planner_dir + "/config/behavior_path_planner_tree.xml");

  test_utils::updateNodeOptions(
    node_options, {planning_test_utils_dir + "/config/test_common.param.yaml",
                   planning_test_utils_dir + "/config/test_nearest_search.param.yaml",
                   planning_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                   behavior_path_planner_dir + "/config/behavior_path_planner.param.yaml",
                   behavior_path_planner_dir + "/config/drivable_area_expansion.param.yaml",
                   behavior_path_planner_dir + "/config/scene_module_manager.param.yaml",
                   behavior_path_planner_dir + "/config/avoidance/avoidance.param.yaml",
                   behavior_path_planner_dir + "/config/lane_change/lane_change.param.yaml",
                   behavior_path_planner_dir + "/config/pull_out/pull_out.param.yaml",
                   behavior_path_planner_dir + "/config/pull_over/pull_over.param.yaml",
                   behavior_path_planner_dir + "/config/avoidance_by_lc/avoidance_by_lc.param.yaml",
                   behavior_path_planner_dir + "/config/side_shift/side_shift.param.yaml"});

  auto test_target_node =
    std::make_shared<behavior_path_planner::BehaviorPathPlannerNode>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishInitialPose(test_target_node, "behavior_path_planner/input/odometry");
  test_manager->publishAcceleration(test_target_node, "behavior_path_planner/input/accel");
  test_manager->publishPredictedObjects(test_target_node, "behavior_path_planner/input/perception");
  test_manager->publishOccupancyGrid(
    test_target_node, "behavior_path_planner/input/occupancy_grid_map");
  test_manager->publishLaneDrivingScenario(
    test_target_node, "behavior_path_planner/input/scenario");
  test_manager->publishMap(test_target_node, "behavior_path_planner/input/vector_map");
  test_manager->publishCostMap(test_target_node, "behavior_path_planner/input/costmap");
  test_manager->publishOperationModeState(test_target_node, "system/operation_mode/state");
  test_manager->publishLateralOffset(
    test_target_node, "behavior_path_planner/input/lateral_offset");

  // set subscriber with topic name: behavior_path_planner → test_node_
  test_manager->setPathWithLaneIdSubscriber("behavior_path_planner/output/path");

  // set behavior_path_planner's input topic name(this topic is changed to test node)
  test_manager->setRouteInputTopicName("behavior_path_planner/input/route");

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithBehaviorNominalRoute(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty route
  test_manager->testWithAbnormalRoute(test_target_node);
  rclcpp::shutdown();
}
