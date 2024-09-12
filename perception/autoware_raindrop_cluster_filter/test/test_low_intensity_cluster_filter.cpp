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

#include "../src/low_intensity_cluster_filter_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_point_types/types.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <vector>

using autoware::low_intensity_cluster_filter::LowIntensityClusterFilter;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_point_types::PointXYZIRC;
using autoware_point_types::PointXYZIRCGenerator;
using point_cloud_msg_wrapper::PointCloud2Modifier;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using tier4_perception_msgs::msg::DetectedObjectWithFeature;

std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<LowIntensityClusterFilter> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto raindrop_cluster_filter_dir =
    ament_index_cpp::get_package_share_directory("autoware_raindrop_cluster_filter");
  node_options.arguments(
    {"--ros-args", "--params-file",
     raindrop_cluster_filter_dir + "/config/low_intensity_cluster_filter.param.yaml"});
  return std::make_shared<LowIntensityClusterFilter>(node_options);
}

DetectedObjectsWithFeature create_cluster(
  const float x, const float y, const float z, const float existence_probability, uint8_t label,
  const int cluster_size, const int intensity)
{
  DetectedObjectsWithFeature msg;
  DetectedObjectWithFeature feature_obj;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.x = x;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.y = y;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.z = z;
  feature_obj.object.existence_probability = existence_probability;
  feature_obj.object.classification.resize(1);
  feature_obj.object.classification[0].label = label;
  sensor_msgs::msg::PointCloud2 cluster;
  PointCloud2Modifier<PointXYZIRC, autoware_point_types::PointXYZIRCGenerator> modifier(
    cluster, "base_link");
  for (int i = 0; i < cluster_size; i++) {
    PointXYZIRC point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    modifier.push_back(std::move(point));
  }
  feature_obj.feature.cluster = cluster;
  msg.feature_objects.push_back(feature_obj);
  msg.header.frame_id = "base_link";
  return msg;
}

TEST(LowIntensityClusterFilterTest, testClusterMergerEmptyObject)
{
  rclcpp::init(0, nullptr);
  const std::string input_cluster = "/input/objects";
  const std::string output_cluster = "/output/objects";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  int counter = 0;
  DetectedObjectsWithFeature latest_msgs;
  auto callback = [&counter, &latest_msgs](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    latest_msgs = *msg;
    ++counter;
  };
  test_manager->set_subscriber<DetectedObjectsWithFeature>(output_cluster, callback);

  DetectedObjectsWithFeature msg;
  msg.header.frame_id = "base_link";
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster, msg);
  EXPECT_EQ(counter, 1);
  EXPECT_EQ(latest_msgs.feature_objects.size(), 0);
  rclcpp::shutdown();
}

TEST(LowIntensityClusterFilterTest, testLowIntensityClusterClusterFilterOut)
{
  rclcpp::init(0, nullptr);
  const std::string input_cluster = "/input/objects";
  const std::string output_cluster = "/output/objects";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  DetectedObjectsWithFeature msg =
    create_cluster(10.0, 5.0, 0.0, 0.0, ObjectClassification::UNKNOWN, 10, 0);
  DetectedObjectsWithFeature latest_msg;
  auto callback = [&latest_msg](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjectsWithFeature>(output_cluster, callback);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster, msg);
  EXPECT_EQ(latest_msg.feature_objects.size(), 0);
  rclcpp::shutdown();
}

TEST(LowIntensityClusterFilterTest, testHightIntensityCluster)
{
  rclcpp::init(0, nullptr);
  const std::string input_cluster = "/input/objects";
  const std::string output_cluster = "/output/objects";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  DetectedObjectsWithFeature msg =
    create_cluster(10.0, 5.0, 0.0, 0.0, ObjectClassification::UNKNOWN, 10, 255);
  DetectedObjectsWithFeature latest_msg;
  auto callback = [&latest_msg](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjectsWithFeature>(output_cluster, callback);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster, msg);
  EXPECT_EQ(latest_msg.feature_objects.size(), 1);
  rclcpp::shutdown();
}

TEST(LowIntensityClusterFilterTest, testHightExistenceProbCluster)
{
  rclcpp::init(0, nullptr);
  const std::string input_cluster = "/input/objects";
  const std::string output_cluster = "/output/objects";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  DetectedObjectsWithFeature msg =
    create_cluster(10.0, 5.0, 0.0, 1.0, ObjectClassification::UNKNOWN, 10, 0);
  DetectedObjectsWithFeature latest_msg;
  auto callback = [&latest_msg](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjectsWithFeature>(output_cluster, callback);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster, msg);
  EXPECT_EQ(latest_msg.feature_objects.size(), 1);
  rclcpp::shutdown();
}

TEST(LowIntensityClusterFilterTest, testKnownClassCluster)
{
  rclcpp::init(0, nullptr);
  const std::string input_cluster = "/input/objects";
  const std::string output_cluster = "/output/objects";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  DetectedObjectsWithFeature msg =
    create_cluster(10.0, 5.0, 0.0, 0.0, ObjectClassification::PEDESTRIAN, 10, 0);
  DetectedObjectsWithFeature latest_msg;
  auto callback = [&latest_msg](const DetectedObjectsWithFeature::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjectsWithFeature>(output_cluster, callback);
  test_manager->test_pub_msg<DetectedObjectsWithFeature>(test_target_node, input_cluster, msg);
  EXPECT_EQ(latest_msg.feature_objects.size(), 1);
  rclcpp::shutdown();
}
