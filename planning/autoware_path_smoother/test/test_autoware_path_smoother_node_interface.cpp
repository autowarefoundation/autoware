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

#include "autoware/path_smoother/elastic_band_smoother.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);

  auto test_manager =
    std::make_shared<autoware::planning_test_manager::PlanningInterfaceTestManager>();

  auto node_options = rclcpp::NodeOptions{};

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto path_smoothing_dir =
    ament_index_cpp::get_package_share_directory("autoware_path_smoother");

  node_options.arguments(
    {"--ros-args", "--params-file", autoware_test_utils_dir + "/config/test_common.param.yaml",
     "--params-file", autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
     "--params-file", path_smoothing_dir + "/config/elastic_band_smoother.param.yaml"});

  auto test_target_node =
    std::make_shared<autoware::path_smoother::ElasticBandSmoother>(node_options);

  // publish necessary topics from test_manager
  test_manager->publishOdometry(test_target_node, "autoware_path_smoother/input/odometry");

  // set subscriber with topic name
  test_manager->setTrajectorySubscriber("autoware_path_smoother/output/traj");
  test_manager->setPathSubscriber("autoware_path_smoother/output/path");

  // set input topic name (this topic is changed to test node)
  test_manager->setPathInputTopicName("autoware_path_smoother/input/path");

  // test with normal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithNominalPath(test_target_node));

  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with trajectory with empty/one point/overlapping point
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithAbnormalPath(test_target_node));

  rclcpp::shutdown();
}
