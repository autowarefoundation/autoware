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
#include "obstacle_velocity_limiter/obstacle_velocity_limiter_node.hpp"
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

  const auto obstacle_velocity_limiter_dir =
    ament_index_cpp::get_package_share_directory("obstacle_velocity_limiter");

  node_options.arguments(
    {"--ros-args", "--params-file",
     planning_test_utils_dir + "/config/test_vehicle_info.param.yaml", "--params-file",
     obstacle_velocity_limiter_dir + "/config/default_obstacle_velocity_limiter.param.yaml"});

  auto test_target_node =
    std::make_shared<obstacle_velocity_limiter::ObstacleVelocityLimiterNode>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "obstacle_velocity_limiter/input/odometry");
  test_manager->publishPointCloud(test_target_node, "obstacle_velocity_limiter/input/pointcloud");
  test_manager->publishOccupancyGrid(
    test_target_node, "obstacle_velocity_limiter/input/occupancy_grid");
  test_manager->publishPredictedObjects(
    test_target_node, "obstacle_velocity_limiter/input/dynamic_obstacles");
  test_manager->publishMap(test_target_node, "obstacle_velocity_limiter/input/map");

  // set subscriber with topic name: obstacle_velocity_limiter → test_node_
  test_manager->setTrajectorySubscriber("obstacle_velocity_limiter/output/trajectory");

  // set obstacle_velocity_limiter's input topic name(this topic is changed to test node):
  test_manager->setTrajectoryInputTopicName("obstacle_velocity_limiter/input/trajectory");

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));

  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  test_manager->testWithAbnormalTrajectory(test_target_node);
}
