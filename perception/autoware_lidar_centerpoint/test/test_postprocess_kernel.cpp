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

#include "test_postprocess_kernel.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::lidar_centerpoint
{

void PostprocessKernelTest::SetUp()
{
  cudaStreamCreate(&stream_);

  constexpr std::size_t class_size{5};
  constexpr std::size_t point_feature_size{4};
  const std::size_t cloud_capacity{2000000};
  constexpr std::size_t max_voxel_size{100000000};
  const std::vector<double> point_cloud_range{-76.8, -76.8, -4.0, 76.8, 76.8, 6.0};
  const std::vector<double> voxel_size{0.32, 0.32, 10.0};
  constexpr std::size_t downsample_factor{1};
  constexpr std::size_t encoder_in_feature_size{9};
  constexpr float score_threshold{0.35f};
  constexpr float circle_nms_dist_threshold{0.5f};
  const std::vector<double> yaw_norm_thresholds{0.5, 0.5, 0.5};
  constexpr bool has_variance{false};

  config_ptr_ = std::make_unique<autoware::lidar_centerpoint::CenterPointConfig>(
    class_size, point_feature_size, cloud_capacity, max_voxel_size, point_cloud_range, voxel_size,
    downsample_factor, encoder_in_feature_size, score_threshold, circle_nms_dist_threshold,
    yaw_norm_thresholds, has_variance);

  postprocess_cuda_ptr_ = std::make_unique<PostProcessCUDA>(*config_ptr_);

  const auto grid_xy_size = config_ptr_->down_grid_size_x_ * config_ptr_->down_grid_size_y_;

  head_out_heatmap_d_ = cuda::make_unique<float[]>(grid_xy_size * config_ptr_->class_size_);
  head_out_offset_d_ =
    cuda::make_unique<float[]>(grid_xy_size * config_ptr_->head_out_offset_size_);
  head_out_z_d_ = cuda::make_unique<float[]>(grid_xy_size * config_ptr_->head_out_z_size_);
  head_out_dim_d_ = cuda::make_unique<float[]>(grid_xy_size * config_ptr_->head_out_dim_size_);
  head_out_rot_d_ = cuda::make_unique<float[]>(grid_xy_size * config_ptr_->head_out_rot_size_);
  head_out_vel_d_ = cuda::make_unique<float[]>(grid_xy_size * config_ptr_->head_out_vel_size_);

  std::vector<float> heatmap_host_vector(grid_xy_size * config_ptr_->class_size_, 0.f);
  std::fill(heatmap_host_vector.begin(), heatmap_host_vector.end(), -1e6);
  cudaMemcpy(
    head_out_heatmap_d_.get(), heatmap_host_vector.data(),
    grid_xy_size * config_ptr_->head_out_offset_size_ * sizeof(float), cudaMemcpyHostToDevice);
  cudaMemset(
    head_out_offset_d_.get(), 0, grid_xy_size * config_ptr_->head_out_offset_size_ * sizeof(float));
  cudaMemset(head_out_z_d_.get(), 0, grid_xy_size * config_ptr_->head_out_z_size_ * sizeof(float));
  cudaMemset(
    head_out_dim_d_.get(), 0, grid_xy_size * config_ptr_->head_out_dim_size_ * sizeof(float));
  cudaMemset(
    head_out_rot_d_.get(), 0, grid_xy_size * config_ptr_->head_out_rot_size_ * sizeof(float));
  cudaMemset(
    head_out_vel_d_.get(), 0, grid_xy_size * config_ptr_->head_out_vel_size_ * sizeof(float));
}

void PostprocessKernelTest::TearDown()
{
}

TEST_F(PostprocessKernelTest, EmptyTensorTest)
{
  std::vector<Box3D> det_boxes3d;

  postprocess_cuda_ptr_->generateDetectedBoxes3D_launch(
    head_out_heatmap_d_.get(), head_out_offset_d_.get(), head_out_z_d_.get(), head_out_dim_d_.get(),
    head_out_rot_d_.get(), head_out_vel_d_.get(), det_boxes3d, stream_);

  ASSERT_EQ(0, det_boxes3d.size());
}

TEST_F(PostprocessKernelTest, SingleDetectionTest)
{
  std::vector<Box3D> det_boxes3d;

  constexpr float detection_x = 70.f;
  constexpr float detection_y = -38.4f;
  constexpr float detection_z = 1.0;
  const float detection_log_w = std::log(7.0);
  const float detection_log_l = std::log(1.0);
  const float detection_log_h = std::log(2.0);
  constexpr float detection_yaw = M_PI_4;
  const float detection_yaw_sin = std::sin(detection_yaw);
  const float detection_yaw_cos = std::sin(detection_yaw);
  constexpr float detection_vel_x = 5.0;
  constexpr float detection_vel_y = -5.0;

  const float idx =
    ((detection_x - config_ptr_->range_min_x_) /
     (config_ptr_->voxel_size_x_ * config_ptr_->downsample_factor_));
  const float idy =
    ((detection_y - config_ptr_->range_min_y_) /
     (config_ptr_->voxel_size_y_ * config_ptr_->downsample_factor_));
  const std::size_t index = config_ptr_->down_grid_size_x_ * std::floor(idy) + std::floor(idx);
  const float detection_x_offset = idx - std::floor(idx);
  const float detection_y_offset = idy - std::floor(idy);

  // Set the values in the cuda tensor
  const auto grid_xy_size = config_ptr_->down_grid_size_x_ * config_ptr_->down_grid_size_y_;
  float score = 1.f;
  cudaMemcpy(
    &head_out_heatmap_d_[2 * grid_xy_size + index], &score, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_offset_d_[0 * grid_xy_size + index], &detection_x_offset, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_offset_d_[1 * grid_xy_size + index], &detection_y_offset, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_z_d_[0 * grid_xy_size + index], &detection_z, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_dim_d_[0 * grid_xy_size + index], &detection_log_w, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_dim_d_[1 * grid_xy_size + index], &detection_log_l, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_dim_d_[2 * grid_xy_size + index], &detection_log_h, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_rot_d_[0 * grid_xy_size + index], &detection_yaw_cos, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_rot_d_[1 * grid_xy_size + index], &detection_yaw_sin, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_vel_d_[0 * grid_xy_size + index], &detection_vel_x, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_vel_d_[1 * grid_xy_size + index], &detection_vel_y, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  auto code = cudaGetLastError();
  ASSERT_EQ(cudaSuccess, code);

  // Extract the boxes
  code = postprocess_cuda_ptr_->generateDetectedBoxes3D_launch(
    head_out_heatmap_d_.get(), head_out_offset_d_.get(), head_out_z_d_.get(), head_out_dim_d_.get(),
    head_out_rot_d_.get(), head_out_vel_d_.get(), det_boxes3d, stream_);

  ASSERT_EQ(cudaSuccess, code);
  ASSERT_EQ(1, det_boxes3d.size());

  const auto det_box3d = det_boxes3d[0];
  EXPECT_EQ(detection_x, det_box3d.x);
  EXPECT_EQ(detection_y, det_box3d.y);
  EXPECT_EQ(detection_z, det_box3d.z);
  EXPECT_NEAR(std::exp(detection_log_w), det_box3d.width, 1e-6);
  EXPECT_NEAR(std::exp(detection_log_l), det_box3d.length, 1e-6);
  EXPECT_NEAR(std::exp(detection_log_h), det_box3d.height, 1e-6);
  EXPECT_EQ(detection_yaw, det_box3d.yaw);
  EXPECT_EQ(detection_vel_x, det_box3d.vel_x);
  EXPECT_EQ(detection_vel_y, det_box3d.vel_y);
}

TEST_F(PostprocessKernelTest, InvalidYawTest)
{
  std::vector<Box3D> det_boxes3d;

  constexpr float detection_x = 70.f;
  constexpr float detection_y = -38.4f;
  constexpr float detection_z = 1.0;
  constexpr float detection_yaw_sin = 0.0;
  constexpr float detection_yaw_cos = 0.2;

  const float idx =
    ((detection_x - config_ptr_->range_min_x_) /
     (config_ptr_->voxel_size_x_ * config_ptr_->downsample_factor_));
  const float idy =
    ((detection_y - config_ptr_->range_min_y_) /
     (config_ptr_->voxel_size_y_ * config_ptr_->downsample_factor_));
  const std::size_t index = config_ptr_->down_grid_size_x_ * std::floor(idy) + std::floor(idx);
  const float detection_x_offset = idx - std::floor(idx);
  const float detection_y_offset = idy - std::floor(idy);

  // Set the values in the cuda tensor
  const auto grid_xy_size = config_ptr_->down_grid_size_x_ * config_ptr_->down_grid_size_y_;
  float score = 1.f;
  cudaMemcpy(
    &head_out_heatmap_d_[2 * grid_xy_size + index], &score, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_offset_d_[0 * grid_xy_size + index], &detection_x_offset, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_offset_d_[1 * grid_xy_size + index], &detection_y_offset, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_z_d_[0 * grid_xy_size + index], &detection_z, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_rot_d_[0 * grid_xy_size + index], &detection_yaw_cos, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_rot_d_[1 * grid_xy_size + index], &detection_yaw_sin, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  auto code = cudaGetLastError();
  ASSERT_EQ(cudaSuccess, code);

  // Extract the boxes
  code = postprocess_cuda_ptr_->generateDetectedBoxes3D_launch(
    head_out_heatmap_d_.get(), head_out_offset_d_.get(), head_out_z_d_.get(), head_out_dim_d_.get(),
    head_out_rot_d_.get(), head_out_vel_d_.get(), det_boxes3d, stream_);

  ASSERT_EQ(cudaSuccess, code);
  ASSERT_EQ(0, det_boxes3d.size());
}

TEST_F(PostprocessKernelTest, CircleNMSTest)
{
  std::vector<Box3D> det_boxes3d;

  constexpr float detection_x = 70.f;
  constexpr float detection_y = -38.4f;
  constexpr float detection_z = 1.0;
  const float detection_log_w = std::log(7.0);
  const float detection_log_l = std::log(1.0);
  const float detection_log_h = std::log(2.0);
  constexpr float detection_yaw1_sin = 0.0;
  constexpr float detection_yaw1_cos = 1.0;
  constexpr float detection_yaw2_sin = 1.0;
  constexpr float detection_yaw2_cos = 0.0;
  constexpr float detection_vel_x = 5.0;
  constexpr float detection_vel_y = -5.0;

  const float idx1 =
    ((detection_x - config_ptr_->range_min_x_) /
     (config_ptr_->voxel_size_x_ * config_ptr_->downsample_factor_));
  const float idy1 =
    ((detection_y - config_ptr_->range_min_y_) /
     (config_ptr_->voxel_size_y_ * config_ptr_->downsample_factor_));
  const std::size_t index1 = config_ptr_->down_grid_size_x_ * std::floor(idy1) + std::floor(idx1);
  const float detection_x_offset1 = idx1 - std::floor(idx1);
  const float detection_y_offset1 = idy1 - std::floor(idy1);

  const float idx2 = idx1 + 1.0;
  const float idy2 = idy1 + 1.0;
  const std::size_t index2 = config_ptr_->down_grid_size_x_ * std::floor(idy2) + std::floor(idx2);
  const float detection_x_offset2 = detection_x_offset1 - 1.0;
  const float detection_y_offset2 = detection_y_offset1 - 1.0;

  // Set the values in the cuda tensor
  const auto grid_xy_size = config_ptr_->down_grid_size_x_ * config_ptr_->down_grid_size_y_;
  float score = 1.f;

  cudaMemcpy(
    &head_out_heatmap_d_[2 * grid_xy_size + index1], &score, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_offset_d_[0 * grid_xy_size + index1], &detection_x_offset1, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_offset_d_[1 * grid_xy_size + index1], &detection_y_offset1, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_z_d_[0 * grid_xy_size + index1], &detection_z, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_dim_d_[0 * grid_xy_size + index1], &detection_log_w, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_dim_d_[1 * grid_xy_size + index1], &detection_log_l, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_dim_d_[2 * grid_xy_size + index1], &detection_log_h, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_rot_d_[0 * grid_xy_size + index1], &detection_yaw1_cos, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_rot_d_[1 * grid_xy_size + index1], &detection_yaw1_sin, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_vel_d_[0 * grid_xy_size + index1], &detection_vel_x, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_vel_d_[1 * grid_xy_size + index1], &detection_vel_y, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_heatmap_d_[2 * grid_xy_size + index2], &score, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_offset_d_[0 * grid_xy_size + index2], &detection_x_offset2, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_offset_d_[1 * grid_xy_size + index2], &detection_y_offset2, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_z_d_[0 * grid_xy_size + index2], &detection_z, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_dim_d_[0 * grid_xy_size + index2], &detection_log_w, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_dim_d_[1 * grid_xy_size + index2], &detection_log_l, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_dim_d_[2 * grid_xy_size + index2], &detection_log_h, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_rot_d_[0 * grid_xy_size + index2], &detection_yaw2_cos, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_rot_d_[1 * grid_xy_size + index2], &detection_yaw2_sin, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  cudaMemcpy(
    &head_out_vel_d_[0 * grid_xy_size + index2], &detection_vel_x, 1 * sizeof(float),
    cudaMemcpyHostToDevice);
  cudaMemcpy(
    &head_out_vel_d_[1 * grid_xy_size + index2], &detection_vel_y, 1 * sizeof(float),
    cudaMemcpyHostToDevice);

  auto code = cudaGetLastError();
  ASSERT_EQ(cudaSuccess, code);

  // Extract the boxes
  code = postprocess_cuda_ptr_->generateDetectedBoxes3D_launch(
    head_out_heatmap_d_.get(), head_out_offset_d_.get(), head_out_z_d_.get(), head_out_dim_d_.get(),
    head_out_rot_d_.get(), head_out_vel_d_.get(), det_boxes3d, stream_);

  ASSERT_EQ(cudaSuccess, code);
  ASSERT_EQ(1, det_boxes3d.size());
}

}  // namespace autoware::lidar_centerpoint

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
