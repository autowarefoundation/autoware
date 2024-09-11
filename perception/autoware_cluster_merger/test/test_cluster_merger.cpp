// Copyright 2024 TIER IV, Inc.
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

#include "../src/cluster_merger_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::cluster_merger::ClusterMergerNode;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<ClusterMergerNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto detected_object_validation_dir =
    ament_index_cpp::get_package_share_directory("autoware_cluster_merger");
  node_options.arguments(
    {"--ros-args", "--params-file",
     detected_object_validation_dir + "/config/cluster_merger.param.yaml"});
  return std::make_shared<ClusterMergerNode>(node_options);
}

TEST(ClusterMergerTest, testClusterMergerEmptyObject)
{
  rclcpp::init(0, nullptr);
  const std::string input_cluster0 = "/input/cluster0";
  const std::string input_cluster1 = "/input/cluster1";
  const std::string output_cluster = "/output/clusters";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  int counter = 0;
  auto callback = [&counter](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    (void)msg;
    ++counter;
  };
  test_manager->set_subscriber<DetectedObjectsWithFeature>(output_cluster, callback);

  DetectedObjectsWithFeature msg0;
  DetectedObjectsWithFeature msg1;
  msg0.header.frame_id = "base_link";
  msg1.header.frame_id = "base_link";

  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster0, msg0);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster1, msg1);

  EXPECT_EQ(counter, 1);

  rclcpp::shutdown();
}

TEST(ClusterMergerTest, testClusterMergerEmptyWithNotEmpty)
{
  rclcpp::init(0, nullptr);
  const std::string input_cluster0 = "/input/cluster0";
  const std::string input_cluster1 = "/input/cluster1";
  const std::string output_cluster = "/output/clusters";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  DetectedObjectsWithFeature msg0;
  DetectedObjectsWithFeature msg1;
  msg0.header.frame_id = "base_link";
  msg1.header.frame_id = "base_link";
  // Create object for cluster 1
  {
    DetectedObjectWithFeature feature_obj;

    feature_obj.object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    feature_obj.object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    feature_obj.object.existence_probability = 1.0f;
    feature_obj.object.classification.resize(1);
    msg1.feature_objects.push_back(feature_obj);
  }

  DetectedObjectsWithFeature latest_msg;
  auto callback = [&latest_msg](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjectsWithFeature>(output_cluster, callback);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster0, msg0);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster1, msg1);
  EXPECT_EQ(latest_msg.feature_objects.size(), 1);
  rclcpp::shutdown();
}

TEST(ClusterMergerTest, testClusterMergerNotEmptyWithNotEmpty)
{
  rclcpp::init(0, nullptr);
  const std::string input_cluster0 = "/input/cluster0";
  const std::string input_cluster1 = "/input/cluster1";
  const std::string output_cluster = "/output/clusters";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  DetectedObjectsWithFeature msg0;
  DetectedObjectsWithFeature msg1;
  msg0.header.frame_id = "base_link";
  msg1.header.frame_id = "base_link";
  // create objects for cluster0

  {
    DetectedObjectWithFeature feature_obj00;

    feature_obj00.object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    feature_obj00.object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    feature_obj00.object.existence_probability = 1.0f;
    feature_obj00.object.classification.resize(1);
    msg0.feature_objects.push_back(feature_obj00);
  }
  {
    DetectedObjectWithFeature feature_obj01;

    feature_obj01.object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    feature_obj01.object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    feature_obj01.object.existence_probability = 1.0f;
    feature_obj01.object.classification.resize(1);
    msg0.feature_objects.push_back(feature_obj01);
  }

  // Create object for cluster 1
  {
    DetectedObjectWithFeature feature_obj10;

    feature_obj10.object.kinematics.pose_with_covariance.pose.position.x = 10.0;
    feature_obj10.object.kinematics.pose_with_covariance.pose.position.y = 5.0;
    feature_obj10.object.existence_probability = 1.0f;
    feature_obj10.object.classification.resize(1);
    msg1.feature_objects.push_back(feature_obj10);
  }

  DetectedObjectsWithFeature latest_msg;
  auto callback = [&latest_msg](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjectsWithFeature>(output_cluster, callback);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster0, msg0);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster1, msg1);
  EXPECT_EQ(latest_msg.feature_objects.size(), 3);
  rclcpp::shutdown();
}
