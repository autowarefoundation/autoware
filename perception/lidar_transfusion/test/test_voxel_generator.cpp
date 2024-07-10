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
#include "lidar_transfusion/transfusion_config.hpp"
#include "lidar_transfusion/utils.hpp"

#include <autoware_point_types/types.hpp>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace lidar_transfusion
{

void VoxelGeneratorTest::SetUp()
{
  // Setup things that should occur before every test instance should go here
  node_ = std::make_shared<rclcpp::Node>("voxel_generator_test_node");

  world_frame_ = "map";
  lidar_frame_ = "lidar";
  delta_pointcloud_x_ = 1.0;
  points_per_pointcloud_ = 300;

  voxels_num_ = std::vector<int64_t>{5000, 30000, 60000};
  point_cloud_range_ = std::vector<double>{-76.8, -76.8, -3.0, 76.8, 76.8, 5.0};
  voxel_size_ = std::vector<double>{0.3, 0.3, 8.0};
  score_threshold_ = 0.2f;
  circle_nms_dist_threshold_ = 0.5f;
  yaw_norm_thresholds_ = std::vector<double>{0.5, 0.5, 0.5};
  config_ptr_ = std::make_unique<TransfusionConfig>(
    voxels_num_, point_cloud_range_, voxel_size_, circle_nms_dist_threshold_, yaw_norm_thresholds_,
    score_threshold_);

  cloud1_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud2_ = std::make_unique<sensor_msgs::msg::PointCloud2>();

  // Set up the fields
  point_cloud_msg_wrapper::PointCloud2Modifier<
    autoware_point_types::PointXYZIRCAEDT, autoware_point_types::PointXYZIRCAEDTGenerator>
    modifier{*cloud1_, lidar_frame_};

  // Resize the cloud to hold points_per_pointcloud_ points
  modifier.resize(points_per_pointcloud_);

  // Create an iterator for desired fields
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud1_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud1_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud1_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_i(*cloud1_, "intensity");

  // Populate the point cloud
  for (size_t i = 0; i < modifier.size(); ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = static_cast<float>(i);
    *iter_y = static_cast<float>(i);
    *iter_z = static_cast<float>(i);
  }
  for (size_t i = 0; i < modifier.size(); ++i, ++iter_i) {
    *iter_i = static_cast<uint8_t>(i % 256);
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

TEST_F(VoxelGeneratorTest, CloudInfo)
{
  CloudInfo cloud_info{};
  EXPECT_EQ(cloud1_->is_bigendian, cloud_info.is_bigendian);
  EXPECT_EQ(cloud1_->fields[0].name, "x");
  EXPECT_EQ(cloud1_->fields[0].datatype, cloud_info.x_datatype);
  EXPECT_EQ(cloud1_->fields[0].count, cloud_info.x_count);
  EXPECT_EQ(cloud1_->fields[0].offset, cloud_info.x_offset);
  EXPECT_EQ(cloud1_->fields[1].name, "y");
  EXPECT_EQ(cloud1_->fields[1].datatype, cloud_info.y_datatype);
  EXPECT_EQ(cloud1_->fields[1].count, cloud_info.y_count);
  EXPECT_EQ(cloud1_->fields[1].offset, cloud_info.y_offset);
  EXPECT_EQ(cloud1_->fields[2].name, "z");
  EXPECT_EQ(cloud1_->fields[2].datatype, cloud_info.z_datatype);
  EXPECT_EQ(cloud1_->fields[2].count, cloud_info.z_count);
  EXPECT_EQ(cloud1_->fields[2].offset, cloud_info.z_offset);
  EXPECT_EQ(cloud1_->fields[3].name, "intensity");
  EXPECT_EQ(cloud1_->fields[3].datatype, cloud_info.intensity_datatype);
  EXPECT_EQ(cloud1_->fields[3].count, cloud_info.intensity_count);
  EXPECT_EQ(cloud1_->fields[3].offset, cloud_info.intensity_offset);
  EXPECT_EQ(cloud1_->width, points_per_pointcloud_);
  EXPECT_EQ(cloud1_->height, 1);
}

TEST_F(VoxelGeneratorTest, SingleFrame)
{
  const unsigned int num_past_frames = 0;  // only current frame
  DensificationParam param(world_frame_, num_past_frames);
  VoxelGenerator voxel_generator(param, *config_ptr_, stream_);
  std::vector<float> points;
  points.resize(config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_);
  std::fill(points.begin(), points.end(), std::nan(""));

  auto points_d =
    cuda::make_unique<float[]>(config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_);
  cudaMemcpy(
    points_d.get(), points.data(),
    points_per_pointcloud_ * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice);

  bool status1 = voxel_generator.enqueuePointCloud(*cloud1_, *tf2_buffer_);
  std::size_t generated_points_num = voxel_generator.generateSweepPoints(*cloud1_, points_d);

  cudaMemcpy(
    points.data(), points_d.get(),
    points_per_pointcloud_ * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyDeviceToHost);

  EXPECT_TRUE(status1);
  EXPECT_EQ(generated_points_num, points_per_pointcloud_);

  // Check valid points
  for (std::size_t i = 0; i < points_per_pointcloud_; ++i) {
    // There are no tf conversions
    EXPECT_EQ(static_cast<double>(i), points[i * config_ptr_->num_point_feature_size_ + 0]);
    EXPECT_EQ(static_cast<double>(i), points[i * config_ptr_->num_point_feature_size_ + 1]);
    EXPECT_EQ(static_cast<double>(i), points[i * config_ptr_->num_point_feature_size_ + 2]);
    EXPECT_EQ(static_cast<double>(i % 256), points[i * config_ptr_->num_point_feature_size_ + 3]);
    EXPECT_EQ(static_cast<double>(0), points[i * config_ptr_->num_point_feature_size_ + 4]);
  }

  // Check invalid points
  for (std::size_t i = points_per_pointcloud_ * config_ptr_->num_point_feature_size_;
       i < points_per_pointcloud_ * config_ptr_->num_point_feature_size_ * 2; ++i) {
    EXPECT_TRUE(std::isnan(points[i]));
  }
}

TEST_F(VoxelGeneratorTest, TwoFramesNoTf)
{
  const unsigned int num_past_frames = 1;
  DensificationParam param(world_frame_, num_past_frames);
  VoxelGenerator voxel_generator(param, *config_ptr_, stream_);
  std::vector<float> points;
  points.resize(config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_);
  std::fill(points.begin(), points.end(), std::nan(""));

  auto points_d =
    cuda::make_unique<float[]>(config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_);
  cudaMemcpy(
    points_d.get(), points.data(),
    points_per_pointcloud_ * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice);

  bool status1 = voxel_generator.enqueuePointCloud(*cloud1_, *tf2_buffer_);
  bool status2 = voxel_generator.enqueuePointCloud(*cloud2_, *tf2_buffer_);
  std::size_t generated_points_num = voxel_generator.generateSweepPoints(*cloud1_, points_d);

  cudaMemcpy(
    points.data(), points_d.get(),
    points_per_pointcloud_ * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyDeviceToHost);

  EXPECT_FALSE(status1);
  EXPECT_FALSE(status2);
  EXPECT_EQ(0, generated_points_num);
}

TEST_F(VoxelGeneratorTest, TwoFrames)
{
  const unsigned int num_past_frames = 1;
  DensificationParam param(world_frame_, num_past_frames);
  VoxelGenerator voxel_generator(param, *config_ptr_, stream_);
  std::vector<float> points;
  points.resize(config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_);
  std::fill(points.begin(), points.end(), std::nan(""));

  auto points_d =
    cuda::make_unique<float[]>(config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_);
  cudaMemcpy(
    points_d.get(), points.data(),
    2 * points_per_pointcloud_ * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice);

  tf2_buffer_->setTransform(transform1_, "authority1");
  tf2_buffer_->setTransform(transform2_, "authority1");

  bool status1 = voxel_generator.enqueuePointCloud(*cloud1_, *tf2_buffer_);
  bool status2 = voxel_generator.enqueuePointCloud(*cloud2_, *tf2_buffer_);
  std::size_t generated_points_num = voxel_generator.generateSweepPoints(*cloud1_, points_d);

  cudaMemcpy(
    points.data(), points_d.get(),
    2 * points_per_pointcloud_ * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyDeviceToHost);

  EXPECT_TRUE(status1);
  EXPECT_TRUE(status2);
  EXPECT_EQ(2 * points_per_pointcloud_, generated_points_num);

  // Check valid points for the latest pointcloud
  for (std::size_t i = 0; i < points_per_pointcloud_; ++i) {
    // There are no tf conversions
    EXPECT_EQ(static_cast<double>(i), points[i * config_ptr_->num_point_feature_size_ + 0]);
    EXPECT_EQ(static_cast<double>(i), points[i * config_ptr_->num_point_feature_size_ + 1]);
    EXPECT_EQ(static_cast<double>(i), points[i * config_ptr_->num_point_feature_size_ + 2]);
    EXPECT_EQ(static_cast<double>(i % 256), points[i * config_ptr_->num_point_feature_size_ + 3]);
    EXPECT_EQ(static_cast<double>(0), points[i * config_ptr_->num_point_feature_size_ + 4]);
  }

  // Check valid points for the oldest pointcloud
  for (std::size_t i = 0; i < points_per_pointcloud_; ++i) {
    // There are tf conversions, so results are not numerically the same
    EXPECT_NEAR(
      static_cast<double>(i) - delta_pointcloud_x_,
      points[(points_per_pointcloud_ + i) * config_ptr_->num_point_feature_size_ + 0], 1e-6);
    EXPECT_NEAR(
      static_cast<double>(i),
      points[(points_per_pointcloud_ + i) * config_ptr_->num_point_feature_size_ + 1], 1e-6);
    EXPECT_NEAR(
      static_cast<double>(i),
      points[(points_per_pointcloud_ + i) * config_ptr_->num_point_feature_size_ + 2], 1e-6);
    EXPECT_NEAR(
      static_cast<double>(i % 256),
      points[(points_per_pointcloud_ + i) * config_ptr_->num_point_feature_size_ + 3], 1e-6);
    EXPECT_NEAR(
      0.1, points[(points_per_pointcloud_ + i) * config_ptr_->num_point_feature_size_ + 4], 1e-6);
  }

  // Check invalid points
  for (std::size_t i = 2 * points_per_pointcloud_ * config_ptr_->num_point_feature_size_;
       i < 3 * points_per_pointcloud_ * config_ptr_->num_point_feature_size_; ++i) {
    EXPECT_TRUE(std::isnan(points[i]));
  }
}
}  // namespace lidar_transfusion

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
