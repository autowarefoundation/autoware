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
#include "obstacle_stop_planner/node.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<planning_test_utils::PlanningInterfaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};

  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");
  const auto obstacle_stop_planner_dir =
    ament_index_cpp::get_package_share_directory("obstacle_stop_planner");

  node_options.append_parameter_override("enable_slow_down", false);

  node_options.arguments(
    {"--ros-args", "--params-file", planning_test_utils_dir + "/config/test_common.param.yaml",
     "--params-file", planning_test_utils_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", planning_test_utils_dir + "/config/test_vehicle_info.param.yaml",
     "--params-file", obstacle_stop_planner_dir + "/config/common.param.yaml", "--params-file",
     obstacle_stop_planner_dir + "/config/adaptive_cruise_control.param.yaml", "--params-file",
     obstacle_stop_planner_dir + "/config/obstacle_stop_planner.param.yaml"});

  auto test_target_node = std::make_shared<motion_planning::ObstacleStopPlannerNode>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "obstacle_stop_planner/input/odometry");
  test_manager->publishPointCloud(test_target_node, "obstacle_stop_planner/input/pointcloud");
  test_manager->publishAcceleration(test_target_node, "obstacle_stop_planner/input/acceleration");
  test_manager->publishPredictedObjects(test_target_node, "obstacle_stop_planner/input/objects");
  test_manager->publishExpandStopRange(
    test_target_node, "obstacle_stop_planner/input/expand_stop_range");

  // set subscriber with topic name: obstacle stop planner → test_node_
  test_manager->setTrajectorySubscriber("obstacle_stop_planner/output/trajectory");

  // set obstacle stop planner's input topic name(this topic is changed to test node):
  test_manager->setTrajectoryInputTopicName("obstacle_stop_planner/input/trajectory");

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));

  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  test_manager->testWithAbnormalTrajectory(test_target_node);
}
