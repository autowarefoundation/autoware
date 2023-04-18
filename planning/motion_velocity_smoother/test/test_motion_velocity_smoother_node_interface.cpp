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
#include "motion_velocity_smoother/motion_velocity_smoother_node.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager.hpp"
#include "planning_interface_test_manager/planning_interface_test_manager_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningModuleInterfaceTest, testPlanningInterfaceWithVariousTrajectoryInput)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<planning_test_utils::PlanningInterfaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};

  node_options.append_parameter_override("algorithm_type", "JerkFiltered");
  node_options.append_parameter_override("publish_debug_trajs", false);

  const auto motion_velocity_smoother_dir =
    ament_index_cpp::get_package_share_directory("motion_velocity_smoother");

  const auto planning_test_utils_dir =
    ament_index_cpp::get_package_share_directory("planning_test_utils");

  node_options.arguments(
    {"--ros-args", "--params-file", planning_test_utils_dir + "/config/test_common.param.yaml",
     "--params-file", planning_test_utils_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", planning_test_utils_dir + "/config/test_vehicle_info.param.yaml",
     "--params-file",
     motion_velocity_smoother_dir + "/config/default_motion_velocity_smoother.param.yaml",
     "--params-file", motion_velocity_smoother_dir + "/config/default_common.param.yaml",
     "--params-file", motion_velocity_smoother_dir + "/config/JerkFiltered.param.yaml"});

  auto test_target_node =
    std::make_shared<motion_velocity_smoother::MotionVelocitySmootherNode>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "/localization/kinematic_state");
  test_manager->publishMaxVelocity(
    test_target_node, "motion_velocity_smoother/input/external_velocity_limit_mps");

  // set subscriber for test_target_node
  test_manager->setTrajectorySubscriber("motion_velocity_smoother/output/trajectory");

  // setting topic name of subscribing topic
  test_manager->setTrajectoryInputTopicName("motion_velocity_smoother/input/trajectory");

  // test for normal trajectory
  ASSERT_NO_THROW(test_manager->testWithNominalTrajectory(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test for trajectory with empty/one point/overlapping point
  test_manager->testWithAbnormalTrajectory(test_target_node);
}
