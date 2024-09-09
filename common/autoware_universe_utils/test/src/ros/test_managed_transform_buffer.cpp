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

#include "autoware/universe_utils/ros/managed_transform_buffer.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

class TestManagedTransformBuffer : public ::testing::Test
{
protected:
  std::shared_ptr<rclcpp::Node> node_{nullptr};
  std::shared_ptr<autoware::universe_utils::ManagedTransformBuffer> managed_tf_buffer_{nullptr};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped tf_base_to_lidar_;
  Eigen::Matrix4f eigen_base_to_lidar_;
  std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_in_;

  geometry_msgs::msg::TransformStamped generateTransformMsg(
    const int32_t seconds, const uint32_t nanoseconds, const std::string & parent_frame,
    const std::string & child_frame, double x, double y, double z, double qx, double qy, double qz,
    double qw)
  {
    rclcpp::Time timestamp(seconds, nanoseconds, RCL_ROS_TIME);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    return tf_msg;
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_managed_transform_buffer");
    managed_tf_buffer_ =
      std::make_shared<autoware::universe_utils::ManagedTransformBuffer>(node_.get(), true);
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

    tf_base_to_lidar_ = generateTransformMsg(
      10, 100'000'000, "base_link", "lidar_top", 0.690, 0.000, 2.100, -0.007, -0.007, 0.692, 0.722);
    eigen_base_to_lidar_ = tf2::transformToEigen(tf_base_to_lidar_).matrix().cast<float>();
    tf_broadcaster_->sendTransform(tf_base_to_lidar_);
    cloud_in_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

    // Set up the fields for x, y, and z coordinates
    cloud_in_->fields.resize(3);
    sensor_msgs::PointCloud2Modifier modifier(*cloud_in_);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    // Resize the cloud to hold points_per_pointcloud_ points
    modifier.resize(10);

    // Create an iterator for the x, y, z fields
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_in_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_in_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_in_, "z");

    // Populate the point cloud
    for (size_t i = 0; i < modifier.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = static_cast<float>(i);
      *iter_y = static_cast<float>(i);
      *iter_z = static_cast<float>(i);
    }

    // Set up cloud header
    cloud_in_->header.frame_id = "lidar_top";
    cloud_in_->header.stamp = rclcpp::Time(10, 100'000'000);

    ASSERT_TRUE(rclcpp::ok());
  }

  void TearDown() override { managed_tf_buffer_.reset(); }
};

TEST_F(TestManagedTransformBuffer, TestTransformNoExist)
{
  Eigen::Matrix4f transform;
  auto success = managed_tf_buffer_->getTransform("base_link", "fake_link", transform);
  EXPECT_TRUE(transform.isIdentity());
  EXPECT_FALSE(success);
}

TEST_F(TestManagedTransformBuffer, TestTransformBase)
{
  Eigen::Matrix4f eigen_base_to_lidar;
  auto success = managed_tf_buffer_->getTransform("base_link", "lidar_top", eigen_base_to_lidar);
  EXPECT_TRUE(eigen_base_to_lidar.isApprox(eigen_base_to_lidar_, 0.001));
  EXPECT_TRUE(success);
}

TEST_F(TestManagedTransformBuffer, TestTransformSameFrame)
{
  Eigen::Matrix4f eigen_base_to_base;
  auto success = managed_tf_buffer_->getTransform("base_link", "base_link", eigen_base_to_base);
  EXPECT_TRUE(eigen_base_to_base.isApprox(Eigen::Matrix4f::Identity(), 0.001));
  EXPECT_TRUE(success);
}

TEST_F(TestManagedTransformBuffer, TestTransformInverse)
{
  Eigen::Matrix4f eigen_lidar_to_base;
  auto success = managed_tf_buffer_->getTransform("lidar_top", "base_link", eigen_lidar_to_base);
  EXPECT_TRUE(eigen_lidar_to_base.isApprox(eigen_base_to_lidar_.inverse(), 0.001));
  EXPECT_TRUE(success);
}

TEST_F(TestManagedTransformBuffer, TestTransformMultipleCall)
{
  Eigen::Matrix4f eigen_transform;
  EXPECT_FALSE(managed_tf_buffer_->getTransform("base_link", "fake_link", eigen_transform));
  EXPECT_TRUE(eigen_transform.isApprox(Eigen::Matrix4f::Identity(), 0.001));
  EXPECT_TRUE(managed_tf_buffer_->getTransform("lidar_top", "base_link", eigen_transform));
  EXPECT_TRUE(eigen_transform.isApprox(eigen_base_to_lidar_.inverse(), 0.001));
  EXPECT_TRUE(managed_tf_buffer_->getTransform("fake_link", "fake_link", eigen_transform));
  EXPECT_TRUE(eigen_transform.isApprox(Eigen::Matrix4f::Identity(), 0.001));
  EXPECT_TRUE(managed_tf_buffer_->getTransform("base_link", "lidar_top", eigen_transform));
  EXPECT_TRUE(eigen_transform.isApprox(eigen_base_to_lidar_, 0.001));
  EXPECT_FALSE(managed_tf_buffer_->getTransform("fake_link", "lidar_top", eigen_transform));
  EXPECT_TRUE(eigen_transform.isApprox(Eigen::Matrix4f::Identity(), 0.001));
  EXPECT_TRUE(managed_tf_buffer_->getTransform("base_link", "lidar_top", eigen_transform));
  EXPECT_TRUE(eigen_transform.isApprox(eigen_base_to_lidar_, 0.001));
}

TEST_F(TestManagedTransformBuffer, TestTransformEmptyPointCloud)
{
  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud_in->header.frame_id = "lidar_top";
  cloud_in->header.stamp = rclcpp::Time(10, 100'000'000);
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out));
  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out));
  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out));
}

TEST_F(TestManagedTransformBuffer, TestTransformEmptyPointCloudNoHeader)
{
  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>();
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out));
  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out));
  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out));
}

TEST_F(TestManagedTransformBuffer, TestTransformPointCloud)
{
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Transform cloud with header
  EXPECT_TRUE(managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in_, *cloud_out));
  EXPECT_TRUE(managed_tf_buffer_->transformPointcloud("base_link", *cloud_in_, *cloud_out));
  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in_, *cloud_out));
}

TEST_F(TestManagedTransformBuffer, TestTransformPointCloudNoHeader)
{
  auto cloud_out = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Transform cloud without header
  auto cloud_in = std::make_unique<sensor_msgs::msg::PointCloud2>(*cloud_in_);
  cloud_in->header.frame_id = "";
  cloud_in->header.stamp = rclcpp::Time(0, 0);
  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("lidar_top", *cloud_in, *cloud_out));
  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("base_link", *cloud_in, *cloud_out));
  EXPECT_FALSE(managed_tf_buffer_->transformPointcloud("fake_link", *cloud_in, *cloud_out));
}
