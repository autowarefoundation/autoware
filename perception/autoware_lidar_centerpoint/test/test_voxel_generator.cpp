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

#include "test_voxel_generator.hpp"

#include "gtest/gtest.h"

#include "sensor_msgs/point_cloud2_iterator.hpp"

void VoxelGeneratorTest::SetUp()
{
  // Setup things that should occur before every test instance should go here
  node_ = std::make_shared<rclcpp::Node>("voxel_generator_test_node");

  world_frame_ = "map";
  lidar_frame_ = "lidar";
  points_per_pointcloud_ = 10;
  capacity_ = 100;
  delta_pointcloud_x_ = 1.0;

  class_size_ = 5;
  point_feature_size_ = 4;
  cloud_capacity_ = 2000000;
  max_voxel_size_ = 100000000;
  point_cloud_range_ = std::vector<double>{-76.8, -76.8, -4.0, 76.8, 76.8, 6.0};
  voxel_size_ = std::vector<double>{0.32, 0.32, 10.0};
  downsample_factor_ = 1;
  encoder_in_feature_size_ = 9;
  score_threshold_ = 0.35f;
  circle_nms_dist_threshold_ = 0.5f;
  yaw_norm_thresholds_ = std::vector<double>{0.5, 0.5, 0.5};
  has_variance_ = false;

  cloud1_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud2_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

  cloud1_->header.frame_id = lidar_frame_;

  // Set up the fields for x, y, and z coordinates
  cloud1_->fields.resize(3);
  sensor_msgs::PointCloud2Modifier modifier(*cloud1_);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  // Resize the cloud to hold points_per_pointcloud_ points
  modifier.resize(points_per_pointcloud_);

  // Create an iterator for the x, y, z fields
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud1_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud1_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud1_, "z");

  // Populate the point cloud
  for (size_t i = 0; i < modifier.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = static_cast<float>(i);
    *iter_y = static_cast<float>(i);
    *iter_z = static_cast<float>(i);
  }

  *cloud2_ = *cloud1_;

  // Set the stamps for the point clouds. They usually come every 100ms
  cloud1_->header.stamp.sec = 1;
  cloud1_->header.stamp.nanosec = 100'000'000;
  cloud2_->header.stamp.sec = 1;
  cloud2_->header.stamp.nanosec = 200'000'000;

  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf2_buffer_->setUsingDedicatedThread(true);

  // The vehicle moves 1m/s in the x direction
  const double world_origin_x = 6'371'000.0;
  const double world_origin_y = 1'371'000.0;

  transform1_.header.stamp = cloud1_->header.stamp;
  transform1_.header.frame_id = world_frame_;
  transform1_.child_frame_id = lidar_frame_;
  transform1_.transform.translation.x = world_origin_x;
  transform1_.transform.translation.y = world_origin_y;
  transform1_.transform.translation.z = 1.8;
  transform1_.transform.rotation.x = 0.0;
  transform1_.transform.rotation.y = 0.0;
  transform1_.transform.rotation.z = 0.0;
  transform1_.transform.rotation.w = 1.0;

  transform2_ = transform1_;
  transform2_.header.stamp = cloud2_->header.stamp;
  transform2_.transform.translation.x = world_origin_x + delta_pointcloud_x_;

  cudaStreamCreate(&stream_);
}

void VoxelGeneratorTest::TearDown()
{
}

TEST_F(VoxelGeneratorTest, SingleFrame)
{
  const unsigned int num_past_frames = 0;  // only current frame

  autoware::lidar_centerpoint::DensificationParam param(world_frame_, num_past_frames);

  autoware::lidar_centerpoint::CenterPointConfig config(
    class_size_, point_feature_size_, cloud_capacity_, max_voxel_size_, point_cloud_range_,
    voxel_size_, downsample_factor_, encoder_in_feature_size_, score_threshold_,
    circle_nms_dist_threshold_, yaw_norm_thresholds_, has_variance_);

  autoware::lidar_centerpoint::VoxelGenerator voxel_generator(param, config);
  std::vector<float> points;
  points.resize(capacity_ * config.point_feature_size_);
  std::fill(points.begin(), points.end(), std::nan(""));

  auto points_d = cuda::make_unique<float[]>(capacity_ * config.point_feature_size_);
  cudaMemcpy(
    points_d.get(), points.data(), capacity_ * config.point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice);

  bool status1 = voxel_generator.enqueuePointCloud(*cloud1_, *tf2_buffer_, stream_);
  std::size_t generated_points_num = voxel_generator.generateSweepPoints(points_d.get(), stream_);

  cudaMemcpy(
    points.data(), points_d.get(), capacity_ * config.point_feature_size_ * sizeof(float),
    cudaMemcpyDeviceToHost);

  EXPECT_TRUE(status1);
  EXPECT_EQ(points_per_pointcloud_, generated_points_num);

  // Check valid points
  for (std::size_t i = 0; i < points_per_pointcloud_; ++i) {
    // There are no tf conversions
    EXPECT_EQ(static_cast<double>(i), points[i * config.point_feature_size_ + 0]);
    EXPECT_EQ(static_cast<double>(i), points[i * config.point_feature_size_ + 1]);
    EXPECT_EQ(static_cast<double>(i), points[i * config.point_feature_size_ + 2]);
    EXPECT_EQ(static_cast<double>(0), points[i * config.point_feature_size_ + 3]);
  }

  // Check invalid points
  for (std::size_t i = points_per_pointcloud_ * config.point_feature_size_;
       i < capacity_ * config.point_feature_size_; ++i) {
    EXPECT_TRUE(std::isnan(points[i]));
  }
}

TEST_F(VoxelGeneratorTest, TwoFramesNoTf)
{
  const unsigned int num_past_frames = 1;

  autoware::lidar_centerpoint::DensificationParam param(world_frame_, num_past_frames);

  autoware::lidar_centerpoint::CenterPointConfig config(
    class_size_, point_feature_size_, cloud_capacity_, max_voxel_size_, point_cloud_range_,
    voxel_size_, downsample_factor_, encoder_in_feature_size_, score_threshold_,
    circle_nms_dist_threshold_, yaw_norm_thresholds_, has_variance_);

  autoware::lidar_centerpoint::VoxelGenerator voxel_generator(param, config);
  std::vector<float> points;
  points.resize(capacity_ * config.point_feature_size_);
  std::fill(points.begin(), points.end(), std::nan(""));

  auto points_d = cuda::make_unique<float[]>(capacity_ * config.point_feature_size_);
  cudaMemcpy(
    points_d.get(), points.data(), capacity_ * config.point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice);

  bool status1 = voxel_generator.enqueuePointCloud(*cloud1_, *tf2_buffer_, stream_);
  bool status2 = voxel_generator.enqueuePointCloud(*cloud2_, *tf2_buffer_, stream_);
  std::size_t generated_points_num = voxel_generator.generateSweepPoints(points_d.get(), stream_);

  cudaMemcpy(
    points.data(), points_d.get(), capacity_ * config.point_feature_size_ * sizeof(float),
    cudaMemcpyDeviceToHost);

  EXPECT_FALSE(status1);
  EXPECT_FALSE(status2);
  EXPECT_EQ(0, generated_points_num);
}

TEST_F(VoxelGeneratorTest, TwoFrames)
{
  const unsigned int num_past_frames = 1;

  autoware::lidar_centerpoint::DensificationParam param(world_frame_, num_past_frames);

  autoware::lidar_centerpoint::CenterPointConfig config(
    class_size_, point_feature_size_, cloud_capacity_, max_voxel_size_, point_cloud_range_,
    voxel_size_, downsample_factor_, encoder_in_feature_size_, score_threshold_,
    circle_nms_dist_threshold_, yaw_norm_thresholds_, has_variance_);

  autoware::lidar_centerpoint::VoxelGenerator voxel_generator(param, config);
  std::vector<float> points;
  points.resize(capacity_ * config.point_feature_size_);
  std::fill(points.begin(), points.end(), std::nan(""));

  auto points_d = cuda::make_unique<float[]>(capacity_ * config.point_feature_size_);
  cudaMemcpy(
    points_d.get(), points.data(), capacity_ * config.point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice);

  tf2_buffer_->setTransform(transform1_, "authority1");
  tf2_buffer_->setTransform(transform2_, "authority1");

  bool status1 = voxel_generator.enqueuePointCloud(*cloud1_, *tf2_buffer_, stream_);
  bool status2 = voxel_generator.enqueuePointCloud(*cloud2_, *tf2_buffer_, stream_);
  std::size_t generated_points_num = voxel_generator.generateSweepPoints(points_d.get(), stream_);

  cudaMemcpy(
    points.data(), points_d.get(), capacity_ * config.point_feature_size_ * sizeof(float),
    cudaMemcpyDeviceToHost);

  EXPECT_TRUE(status1);
  EXPECT_TRUE(status2);
  EXPECT_EQ(2 * points_per_pointcloud_, generated_points_num);

  // Check valid points for the latest pointcloud
  for (std::size_t i = 0; i < points_per_pointcloud_; ++i) {
    // There are no tf conversions
    EXPECT_EQ(static_cast<double>(i), points[i * config.point_feature_size_ + 0]);
    EXPECT_EQ(static_cast<double>(i), points[i * config.point_feature_size_ + 1]);
    EXPECT_EQ(static_cast<double>(i), points[i * config.point_feature_size_ + 2]);
    EXPECT_EQ(static_cast<double>(0), points[i * config.point_feature_size_ + 3]);
  }

  // Check valid points for the oldest pointcloud
  for (std::size_t i = 0; i < points_per_pointcloud_; ++i) {
    // There are tf conversions, so results are not numerically the same
    EXPECT_NEAR(
      static_cast<double>(i) - delta_pointcloud_x_,
      points[(points_per_pointcloud_ + i) * config.point_feature_size_ + 0], 1e-6);
    EXPECT_NEAR(
      static_cast<double>(i), points[(points_per_pointcloud_ + i) * config.point_feature_size_ + 1],
      1e-6);
    EXPECT_NEAR(
      static_cast<double>(i), points[(points_per_pointcloud_ + i) * config.point_feature_size_ + 2],
      1e-6);
    EXPECT_NEAR(0.1, points[(points_per_pointcloud_ + i) * config.point_feature_size_ + 3], 1e-6);
  }

  // Check invalid points
  for (std::size_t i = 2 * points_per_pointcloud_ * config.point_feature_size_;
       i < capacity_ * config.point_feature_size_; ++i) {
    EXPECT_TRUE(std::isnan(points[i]));
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
