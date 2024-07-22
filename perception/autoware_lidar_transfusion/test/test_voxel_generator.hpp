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

#ifndef TEST_VOXEL_GENERATOR_HPP_
#define TEST_VOXEL_GENERATOR_HPP_

#include <autoware/lidar_transfusion/preprocess/voxel_generator.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::lidar_transfusion
{

class VoxelGeneratorTest : public testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

  // These need to be public so that they can be accessed in the test cases
  rclcpp::Node::SharedPtr node_{};

  std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud1_{}, cloud2_{};
  geometry_msgs::msg::TransformStamped transform1_{}, transform2_{};

  std::unique_ptr<DensificationParam> densification_param_ptr_{};
  std::unique_ptr<TransfusionConfig> config_ptr_{};

  std::unique_ptr<tf2_ros::Buffer> tf2_buffer_{};

  std::string world_frame_{};
  std::string lidar_frame_{};
  double delta_pointcloud_x_{};
  std::size_t points_per_pointcloud_{};

  std::size_t cloud_capacity_{};
  std::vector<int64_t> voxels_num_{};
  std::vector<double> point_cloud_range_{};
  std::vector<double> voxel_size_{};
  std::size_t num_proposals_{};
  float circle_nms_dist_threshold_{};
  std::vector<double> yaw_norm_thresholds_{};
  float score_threshold_{};

  cudaStream_t stream_{};
};
}  // namespace autoware::lidar_transfusion

#endif  // TEST_VOXEL_GENERATOR_HPP_
