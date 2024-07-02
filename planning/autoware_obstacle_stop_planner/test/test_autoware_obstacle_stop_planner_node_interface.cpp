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

#include "node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::motion_planning::ObstacleStopPlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();
  test_manager->setTrajectorySubscriber("obstacle_stop_planner/output/trajectory");
  test_manager->setTrajectoryInputTopicName("obstacle_stop_planner/input/trajectory");
  test_manager->setOdometryTopicName("obstacle_stop_planner/input/odometry");
  return test_manager;
}

std::shared_ptr<ObstacleStopPlannerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto obstacle_stop_planner_dir =
    ament_index_cpp::get_package_share_directory("autoware_obstacle_stop_planner");

  node_options.append_parameter_override("enable_slow_down", false);

  node_options.arguments(
    {"--ros-args", "--params-file", autoware_test_utils_dir + "/config/test_common.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
     "--params-file", obstacle_stop_planner_dir + "/config/common.param.yaml", "--params-file",
     obstacle_stop_planner_dir + "/config/adaptive_cruise_control.param.yaml", "--params-file",
     obstacle_stop_planner_dir + "/config/obstacle_stop_planner.param.yaml"});

  return std::make_shared<ObstacleStopPlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<ObstacleStopPlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "obstacle_stop_planner/input/odometry");
  test_manager->publishPointCloud(test_target_node, "obstacle_stop_planner/input/pointcloud");
  test_manager->publishAcceleration(test_target_node, "obstacle_stop_planner/input/acceleration");
  test_manager->publishPredictedObjects(test_target_node, "obstacle_stop_planner/input/objects");
  test_manager->publishExpandStopRange(
    test_target_node, "obstacle_stop_planner/input/expand_stop_range");
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
  test_manager->testWithAbnormalTrajectory(test_target_node);

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
