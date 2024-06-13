// Copyright 2024 Tier IV, Inc.
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
#include <autoware_planning_test_manager/autoware_planning_test_manager_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::motion_velocity_planner::MotionVelocityPlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;

std::shared_ptr<PlanningInterfaceTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<PlanningInterfaceTestManager>();

  // set subscriber with topic name: motion_velocity_planner → test_node_
  test_manager->setTrajectorySubscriber("motion_velocity_planner_node/output/trajectory");

  // set motion_velocity_planner node's input topic name(this topic is changed to test node)
  test_manager->setTrajectoryInputTopicName("motion_velocity_planner_node/input/trajectory");

  test_manager->setInitialPoseTopicName("motion_velocity_planner_node/input/vehicle_odometry");
  test_manager->setOdometryTopicName("motion_velocity_planner_node/input/vehicle_odometry");

  return test_manager;
}

std::shared_ptr<MotionVelocityPlannerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};

  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");
  const auto motion_velocity_planner_dir =
    ament_index_cpp::get_package_share_directory("autoware_motion_velocity_planner_node");
  const auto velocity_smoother_dir =
    ament_index_cpp::get_package_share_directory("autoware_velocity_smoother");

  const auto get_motion_velocity_module_config = [](const std::string & module) {
    const auto package_name = "autoware_motion_velocity_" + module + "_module";
    const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
    return package_path + "/config/" + module + ".param.yaml";
  };

  std::vector<std::string> module_names;
  module_names.emplace_back("autoware::motion_velocity_planner::OutOfLaneModule");
  module_names.emplace_back("autoware::motion_velocity_planner::ObstacleVelocityLimiterModule");
  module_names.emplace_back("autoware::motion_velocity_planner::DynamicObstacleStopModule");

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("launch_modules", module_names);
  params.emplace_back("is_simulation", false);
  node_options.parameter_overrides(params);

  autoware::test_utils::updateNodeOptions(
    node_options, {autoware_test_utils_dir + "/config/test_common.param.yaml",
                   autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
                   autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
                   velocity_smoother_dir + "/config/default_velocity_smoother.param.yaml",
                   velocity_smoother_dir + "/config/Analytical.param.yaml",
                   motion_velocity_planner_dir + "/config/motion_velocity_planner.param.yaml",
                   get_motion_velocity_module_config("out_of_lane"),
                   get_motion_velocity_module_config("obstacle_velocity_limiter"),
                   get_motion_velocity_module_config("dynamic_obstacle_stop")});

  return std::make_shared<MotionVelocityPlannerNode>(node_options);
}

void publishMandatoryTopics(
  std::shared_ptr<PlanningInterfaceTestManager> test_manager,
  std::shared_ptr<MotionVelocityPlannerNode> test_target_node)
{
  // publish necessary topics from test_manager
  test_manager->publishTF(test_target_node, "/tf");
  test_manager->publishAcceleration(test_target_node, "motion_velocity_planner_node/input/accel");
  test_manager->publishPredictedObjects(
    test_target_node, "motion_velocity_planner_node/input/dynamic_objects");
  test_manager->publishPointCloud(
    test_target_node, "motion_velocity_planner_node/input/no_ground_pointcloud");
  test_manager->publishOdometry(
    test_target_node, "motion_velocity_planner_node/input/vehicle_odometry");
  test_manager->publishAcceleration(test_target_node, "motion_velocity_planner_node/input/accel");
  test_manager->publishMap(test_target_node, "motion_velocity_planner_node/input/vector_map");
  test_manager->publishTrafficSignals(
    test_target_node, "motion_velocity_planner_node/input/traffic_signals");
  test_manager->publishVirtualTrafficLightState(
    test_target_node, "motion_velocity_planner_node/input/virtual_traffic_light_states");
  test_manager->publishOccupancyGrid(
    test_target_node, "motion_velocity_planner_node/input/occupancy_grid");
}

TEST(PlanningModuleInterfaceTest, NodeTestWithExceptionTrajectory)
{
  rclcpp::init(0, nullptr);
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  publishMandatoryTopics(test_manager, test_target_node);

  // test with nominal trajectory
  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testWithNominalTrajectory(test_target_node));
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  // test with empty trajectory
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

  // make sure motion_velocity_planner is running
  EXPECT_GE(test_manager->getReceivedTopicNum(), 1);

  ASSERT_NO_THROW_WITH_ERROR_MSG(test_manager->testOffTrackFromTrajectory(test_target_node));

  rclcpp::shutdown();
}
