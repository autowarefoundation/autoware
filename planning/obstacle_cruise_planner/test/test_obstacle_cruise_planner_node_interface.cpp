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

#include "obstacle_cruise_planner/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <planning_test_utils/planning_interface_test_manager.hpp>
#include <planning_test_utils/planning_interface_test_manager_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using motion_planning::ObstacleCruisePlannerNode;
using planning_test_utils::PlanningInterfaceTestManager;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: obstacle_cruise_planner → test_node_
  test_manager->setTrajectorySubscriber("obstacle_cruise_planner/output/trajectory");

  // set obstacle_cruise_planners input topic name(this topic is changed to test node):
  test_manager->setTrajectoryInputTopicName("obstacle_cruise_planner/input/trajectory");

  test_manager->setOdometryTopicName("obstacle_cruise_planner/input/odometry");

  return test_manager;
}

std::shared_ptr<ObstacleCruisePlannerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto obstacle_cruise_planner_dir =
    ament_index_cpp::get_package_share_directory("obstacle_cruise_planner");
  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  node_options.arguments(
    {"--ros-args", "--params-file", planning_test_utils_dir + "/config/test_common.param.yaml",
     "--params-file", planning_test_utils_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", planning_test_utils_dir + "/config/test_vehicle_info.param.yaml",
     "--params-file", obstacle_cruise_planner_dir + "/config/default_common.param.yaml",
     "--params-file", obstacle_cruise_planner_dir + "/config/obstacle_cruise_planner.param.yaml"});

  return std::make_shared<motion_planning::ObstacleCruisePlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<ObstacleCruisePlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "obstacle_cruise_planner/input/odometry");
  test_manager->publishPredictedObjects(test_target_node, "obstacle_cruise_planner/input/objects");
  test_manager->publishAcceleration(test_target_node, "obstacle_cruise_planner/input/acceleration");
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithNominalTrajectory(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithAbnormalTrajectory(test_target_node));

  rclcpp::shutdown();
}

TEST(PlanningModuleInterfaceTest, NodeTestWithOffTrackEgoPose)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  // test for normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithNominalTrajectory(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testTrajectoryWithInvalidEgoPose(test_target_node));

  rclcpp::shutdown();
}
