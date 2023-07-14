// Copyright 2023 The Autoware Contributors
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

#include "../src/pointcloud_map_loader/pointcloud_map_loader_module.hpp"
#include "../src/pointcloud_map_loader/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <chrono>
#include <memory>

using std::chrono_literals::operator""ms;

class TestPointcloudMapLoaderModule : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node;
  std::string temp_pcd_path;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node = rclcpp::Node::make_shared("test_pointcloud_map_loader_module");

    // Create a temporary PCD file with dummy point cloud data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud.points[i].x = static_cast<float>(i);
      cloud.points[i].y = static_cast<float>(i * 2);
      cloud.points[i].z = static_cast<float>(i * 3);
    }

    temp_pcd_path = "/tmp/test_pointcloud_map_loader_module.pcd";
    pcl::io::savePCDFileASCII(temp_pcd_path, cloud);
  }

  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TestPointcloudMapLoaderModule, LoadPCDFilesNoDownsampleTest)
{
  // Prepare PCD paths
  std::vector<std::string> pcd_paths = {temp_pcd_path};

  // Create PointcloudMapLoaderModule instance
  PointcloudMapLoaderModule loader(node.get(), pcd_paths, "pointcloud_map_no_downsample", false);

  // Create a subscriber to the published pointcloud topic
  auto pointcloud_received = std::make_shared<bool>(false);
  auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

  rclcpp::QoS durable_qos{10};
  durable_qos.transient_local();  // Match publisher's Durability setting

  auto pointcloud_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map_no_downsample", durable_qos,
    [pointcloud_received, pointcloud_msg](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      *pointcloud_received = true;
      *pointcloud_msg = *msg;
    });

  // Spin until pointcloud is received or timeout occurs
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto start_time = node->now();
  while (!*pointcloud_received && (node->now() - start_time).seconds() < 3) {
    executor.spin_some(50ms);
  }

  // Check if the point cloud is received and the content is as expected
  ASSERT_TRUE(*pointcloud_received);

  // Convert the received point cloud to pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZ> received_cloud;
  pcl::fromROSMsg(*pointcloud_msg, received_cloud);

  // Check if the received point cloud matches the content of the temporary PCD file
  ASSERT_EQ(static_cast<int>(received_cloud.width), 5);
  ASSERT_EQ(static_cast<int>(received_cloud.height), 1);

  for (size_t i = 0; i < received_cloud.points.size(); ++i) {
    EXPECT_FLOAT_EQ(received_cloud.points[i].x, static_cast<float>(i));
    EXPECT_FLOAT_EQ(received_cloud.points[i].y, static_cast<float>(i * 2));
    EXPECT_FLOAT_EQ(received_cloud.points[i].z, static_cast<float>(i * 3));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
