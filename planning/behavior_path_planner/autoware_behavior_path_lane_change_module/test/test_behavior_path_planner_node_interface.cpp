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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autoware/behavior_path_planner/behavior_path_planner_node.hpp"
#include "autoware_planning_test_manager/autoware_planning_test_manager.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::behavior_path_planner::BehaviorPathPlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: behavior_path_planner → test_node_
  test_manager->setPathWithLaneIdSubscriber("behavior_path_planner/output/path");

  // set behavior_path_planner's input topic name(this topic is changed to test node)
  test_manager->setRouteInputTopicName("behavior_path_planner/input/route");

  test_manager->setInitialPoseTopicName("behavior_path_planner/input/odometry");

  return test_manager;
}

std::shared_ptr<BehaviorPathPlannerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto behavior_path_planner_dir =
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner");
  const auto behavior_path_lane_change_module_dir =
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_lane_change_module");

  std::vector<std::string> module_names;
  module_names.emplace_back("autoware::behavior_path_planner::LaneChangeRightModuleManager");
  module_names.emplace_back("autoware::behavior_path_planner::LaneChangeLeftModuleManager");

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("launch_modules", module_names);
  node_options.parameter_overrides(params);

  autoware::test_utils::updateNodeOptions(
    node_options, {autoware_test_utils_dir + "/config/test_common.param.yaml",
                   autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
                   autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                   behavior_path_planner_dir + "/config/behavior_path_planner.param.yaml",
                   behavior_path_planner_dir + "/config/drivable_area_expansion.param.yaml",
                   behavior_path_planner_dir + "/config/scene_module_manager.param.yaml",
                   behavior_path_lane_change_module_dir + "/config/lane_change.param.yaml"});

  return std::make_shared<BehaviorPathPlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<BehaviorPathPlannerNode> test_target_node)
{
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
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionRoute)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithBehaviorNominalRoute(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty route
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithAbnormalRoute(test_target_node));
  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();
  publishMandatoryTopics(test_manager, test_target_node);

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithBehaviorNominalRoute(test_target_node));

  // make sure behavior_path_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testRouteWithInvalidEgoPose(test_target_node));

  rclcpp::shutdown();
}
