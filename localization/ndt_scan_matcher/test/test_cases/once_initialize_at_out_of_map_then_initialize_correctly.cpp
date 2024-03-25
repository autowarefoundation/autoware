// Copyright 2023 Autoware Foundation
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

#ifndef TEST_CASES__ILLEGAL_INITIAL_POSE_ESTIMATION_HPP_
#define TEST_CASES__ILLEGAL_INITIAL_POSE_ESTIMATION_HPP_

#include "../test_fixture.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <memory>
#include <string>
#include <vector>

TEST_F(TestNDTScanMatcher, once_initialize_at_out_of_map_then_initialize_correctly)  // NOLINT
{
  //---------//
  // Arrange //
  //---------//
  std::thread t1([&]() {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
  std::thread t2([&]() { rclcpp::spin(pcd_loader_); });

  //-----//
  // Act //
  //-----//
  // (1) trigger initial pose estimation
  EXPECT_TRUE(trigger_node_client_->send_trigger_node(true));

  // (2) publish LiDAR point cloud
  const sensor_msgs::msg::PointCloud2 input_cloud = make_default_sensor_pcd();
  RCLCPP_INFO_STREAM(node_->get_logger(), "sensor cloud size: " << input_cloud.width);
  sensor_pcd_publisher_->publish_pcd(input_cloud);

  // (3) send initial pose at out of map. Must not crash.
  const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg_out =
    make_pose(/* x = */ -100.0, /* y = */ -100.0);
  initialpose_client_->send_initialpose(initial_pose_msg_out);

  // (4) send initial pose at correct position
  const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg_correct =
    make_pose(/* x = */ 100.0, /* y = */ 100.0);
  const geometry_msgs::msg::Pose result_pose =
    initialpose_client_->send_initialpose(initial_pose_msg_correct).pose.pose;

  //--------//
  // Assert //
  //--------//
  RCLCPP_INFO_STREAM(
    node_->get_logger(), std::fixed << "result_pose: " << result_pose.position.x << ", "
                                    << result_pose.position.y << ", " << result_pose.position.z);
  EXPECT_NEAR(result_pose.position.x, 100.0, 2.0);
  EXPECT_NEAR(result_pose.position.y, 100.0, 2.0);
  EXPECT_NEAR(result_pose.position.z, 0.0, 2.0);

  rclcpp::shutdown();
  t1.join();
  t2.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

#endif  // TEST_CASES__ILLEGAL_INITIAL_POSE_ESTIMATION_HPP_
