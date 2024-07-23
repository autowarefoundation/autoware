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

#include "test_preprocess_kernel.hpp"

#include <autoware/lidar_centerpoint/preprocess/preprocess_kernel.hpp>

#include <gtest/gtest.h>
#include <thrust/host_vector.h>
#include <thrust/reduce.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::lidar_centerpoint
{

void PreprocessKernelTest::SetUp()
{
  cudaStreamCreate(&stream_);

  max_voxel_size_ = 40000;
  max_point_in_voxel_size_ =
    32;  // this value is actually hardcoded in the kernel so it can not be changed
  point_feature_size_ = 4;
  point_dim_size_ = 3;
  config_encoder_in_feature_size_ = 9;
  encoder_out_feature_size_ = 32;
  capacity_ = 1000000;

  range_min_x_ = -76.8f;
  range_min_y_ = -76.8f;
  range_min_z_ = -4.0f;
  range_max_x_ = 76.8f;
  range_max_y_ = 76.8f;
  range_max_z_ = 6.0f;
  voxel_size_x_ = 0.32f;
  voxel_size_y_ = 0.32f;
  voxel_size_z_ = 10.0f;

  grid_size_x_ = static_cast<std::size_t>((range_max_x_ - range_min_x_) / voxel_size_x_);
  grid_size_y_ = static_cast<std::size_t>((range_max_y_ - range_min_y_) / voxel_size_y_);
  grid_size_z_ = static_cast<std::size_t>((range_max_z_ - range_min_z_) / voxel_size_z_);

  voxels_size_ = max_voxel_size_ * max_point_in_voxel_size_ * point_feature_size_;
  coordinates_size_ = max_voxel_size_ * point_dim_size_;
  encoder_in_feature_size_ =
    max_voxel_size_ * max_point_in_voxel_size_ * config_encoder_in_feature_size_;
  pillar_features_size_ = max_voxel_size_ * encoder_out_feature_size_;
  spatial_features_size_ = grid_size_x_ * grid_size_y_ * encoder_out_feature_size_;
  grid_xy_size_ = grid_size_x_ * grid_size_y_;

  voxels_buffer_size_ =
    grid_size_x_ * grid_size_y_ * max_point_in_voxel_size_ * point_feature_size_;
  mask_size_ = grid_size_x_ * grid_size_y_;

  voxels_d_ = cuda::make_unique<float[]>(voxels_size_);
  coordinates_d_ = cuda::make_unique<int[]>(coordinates_size_);
  num_points_per_voxel_d_ = cuda::make_unique<float[]>(max_voxel_size_);
  encoder_in_features_d_ = cuda::make_unique<float[]>(encoder_in_feature_size_);
  pillar_features_d_ = cuda::make_unique<float[]>(pillar_features_size_);
  spatial_features_d_ = cuda::make_unique<float[]>(spatial_features_size_);
  points_d_ = cuda::make_unique<float[]>(capacity_ * point_feature_size_);
  voxels_buffer_d_ = cuda::make_unique<float[]>(voxels_buffer_size_);
  mask_d_ = cuda::make_unique<unsigned int[]>(mask_size_);
  num_voxels_d_ = cuda::make_unique<unsigned int[]>(1);

  CHECK_CUDA_ERROR(cudaMemsetAsync(num_voxels_d_.get(), 0, sizeof(unsigned int), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(voxels_buffer_d_.get(), 0, voxels_buffer_size_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(mask_d_.get(), 0, mask_size_ * sizeof(int), stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(voxels_d_.get(), 0, voxels_size_ * sizeof(float), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(coordinates_d_.get(), 0, coordinates_size_ * sizeof(int), stream_));
  CHECK_CUDA_ERROR(
    cudaMemsetAsync(num_points_per_voxel_d_.get(), 0, max_voxel_size_ * sizeof(float), stream_));
}

void PreprocessKernelTest::TearDown()
{
}

TEST_F(PreprocessKernelTest, EmptyVoxelTest)
{
  std::vector<float> points{};
  std::size_t points_num = 0;

  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), points_num * point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  cudaError_t code = autoware::lidar_centerpoint::generateVoxels_random_launch(
    points_d_.get(), points_num, range_min_x_, range_max_x_, range_min_y_, range_max_y_,
    range_min_z_, range_max_z_, voxel_size_x_, voxel_size_y_, voxel_size_z_, grid_size_y_,
    grid_size_x_, mask_d_.get(), voxels_buffer_d_.get(), stream_);

  ASSERT_EQ(cudaSuccess, code);

  // Compute the total amount of voxels filled
  thrust::host_vector<unsigned int> mask_h(mask_size_);

  CHECK_CUDA_ERROR(cudaMemcpy(
    mask_h.data(), mask_d_.get(), mask_size_ * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  int sum = thrust::reduce(mask_h.begin(), mask_h.end(), 0);
  EXPECT_EQ(0, sum);
}

TEST_F(PreprocessKernelTest, BasicTest)
{
  std::vector<float> points{25.f, -61.1f, 1.8f, 0.1f};
  std::size_t points_num = 1;

  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), points_num * point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  cudaError_t code = autoware::lidar_centerpoint::generateVoxels_random_launch(
    points_d_.get(), points_num, range_min_x_, range_max_x_, range_min_y_, range_max_y_,
    range_min_z_, range_max_z_, voxel_size_x_, voxel_size_y_, voxel_size_z_, grid_size_y_,
    grid_size_x_, mask_d_.get(), voxels_buffer_d_.get(), stream_);

  ASSERT_EQ(cudaSuccess, code);

  // Compute the total amount of voxels filled
  thrust::host_vector<unsigned int> mask_h(mask_size_);

  CHECK_CUDA_ERROR(cudaMemcpy(
    mask_h.data(), mask_d_.get(), mask_size_ * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  int sum = thrust::reduce(mask_h.begin(), mask_h.end(), 0);
  EXPECT_EQ(1, sum);

  // Check that the voxel was filled
  int voxel_idx = std::floor((points[0] - range_min_x_) / voxel_size_x_);
  int voxel_idy = std::floor((points[1] - range_min_y_) / voxel_size_y_);
  unsigned int voxel_index = voxel_idy * grid_size_x_ + voxel_idx;

  unsigned voxel_count;
  std::array<float, 4> voxel_data{0.f, 0.f, 0.f, 0.f};

  CHECK_CUDA_ERROR(cudaMemcpy(
    &voxel_count, mask_d_.get() + voxel_index, 1 * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  EXPECT_EQ(1, voxel_count);

  CHECK_CUDA_ERROR(cudaMemcpy(
    voxel_data.data(),
    voxels_buffer_d_.get() + voxel_index * max_point_in_voxel_size_ * point_feature_size_,
    point_feature_size_ * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  EXPECT_EQ(points[0], voxel_data[0]);
  EXPECT_EQ(points[1], voxel_data[1]);
  EXPECT_EQ(points[2], voxel_data[2]);
  EXPECT_EQ(points[3], voxel_data[3]);

  code = generateBaseFeatures_launch(
    mask_d_.get(), voxels_buffer_d_.get(), grid_size_y_, grid_size_x_, max_voxel_size_,
    num_voxels_d_.get(), voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(),
    stream_);

  ASSERT_EQ(cudaSuccess, code);

  unsigned int num_pillars{};
  std::vector<float> voxel_features(point_feature_size_, 0.f);
  float num_voxels{};
  std::vector<int> voxel_coordinates(3, 0);

  CHECK_CUDA_ERROR(cudaMemcpy(
    &num_pillars, num_voxels_d_.get(), 1 * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  CHECK_CUDA_ERROR(cudaMemcpy(
    voxel_features.data(), voxels_d_.get(), point_feature_size_ * sizeof(unsigned int),
    cudaMemcpyDeviceToHost));

  CHECK_CUDA_ERROR(cudaMemcpy(
    &num_voxels, num_points_per_voxel_d_.get(), 1 * sizeof(float), cudaMemcpyDeviceToHost));

  CHECK_CUDA_ERROR(cudaMemcpy(
    voxel_coordinates.data(), coordinates_d_.get(), 3 * sizeof(int), cudaMemcpyDeviceToHost));

  EXPECT_EQ(1, num_pillars);
  EXPECT_EQ(1.0, num_voxels);
  EXPECT_EQ(0, voxel_coordinates[0]);
  EXPECT_EQ(voxel_idy, voxel_coordinates[1]);
  EXPECT_EQ(voxel_idx, voxel_coordinates[2]);
  EXPECT_EQ(points[0], voxel_features[0]);
  EXPECT_EQ(points[1], voxel_features[1]);
  EXPECT_EQ(points[2], voxel_features[2]);
  EXPECT_EQ(points[3], voxel_features[3]);

  code = generateFeatures_launch(
    voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(), num_voxels_d_.get(),
    max_voxel_size_, voxel_size_x_, voxel_size_y_, voxel_size_z_, range_min_x_, range_min_y_,
    range_min_z_, encoder_in_features_d_.get(), stream_);

  ASSERT_EQ(cudaSuccess, code);

  std::vector<float> encoder_features(config_encoder_in_feature_size_, 0.f);

  CHECK_CUDA_ERROR(cudaMemcpy(
    encoder_features.data(), encoder_in_features_d_.get(),
    config_encoder_in_feature_size_ * sizeof(float), cudaMemcpyDeviceToHost));

  // The first four values are just the point features
  EXPECT_EQ(points[0], encoder_features[0]);
  EXPECT_EQ(points[1], encoder_features[1]);
  EXPECT_EQ(points[2], encoder_features[2]);
  EXPECT_EQ(points[3], encoder_features[3]);

  // The next three values are the relative coordinates with respect to the voxel average
  EXPECT_EQ(0.0, encoder_features[4]);
  EXPECT_EQ(0.0, encoder_features[5]);
  EXPECT_EQ(0.0, encoder_features[6]);

  // The last two values are the relative coordinates with respect to the voxel center
  float voxel_x_offset = voxel_size_x_ / 2 + voxel_idx * voxel_size_x_ + range_min_x_;
  float voxel_y_offset = voxel_size_y_ / 2 + voxel_idy * voxel_size_y_ + range_min_y_;

  EXPECT_EQ(points[0] - voxel_x_offset, encoder_features[7]);
  EXPECT_EQ(points[1] - voxel_y_offset, encoder_features[8]);
}

TEST_F(PreprocessKernelTest, OutOfRangeTest)
{
  std::vector<float> points{25.f, -61.1f, 100.f, 0.1f};  // 100.f is out of range
  std::size_t points_num = 1;

  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), points_num * point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  cudaError_t code = autoware::lidar_centerpoint::generateVoxels_random_launch(
    points_d_.get(), points_num, range_min_x_, range_max_x_, range_min_y_, range_max_y_,
    range_min_z_, range_max_z_, voxel_size_x_, voxel_size_y_, voxel_size_z_, grid_size_y_,
    grid_size_x_, mask_d_.get(), voxels_buffer_d_.get(), stream_);

  ASSERT_EQ(cudaSuccess, code);

  // Compute the total amount of voxels filled
  thrust::host_vector<unsigned int> mask_h(mask_size_);

  CHECK_CUDA_ERROR(cudaMemcpy(
    mask_h.data(), mask_d_.get(), mask_size_ * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  int sum = thrust::reduce(mask_h.begin(), mask_h.end(), 0);
  EXPECT_EQ(0, sum);
}

TEST_F(PreprocessKernelTest, VoxelOverflowTest)
{
  std::array<float, 4> point{25.f, -61.1f, 1.8f, 0.1f};
  std::size_t points_num = 64;
  std::vector<float> points{};

  for (std::size_t i = 0; i < points_num; ++i) {
    points.insert(points.end(), point.begin(), point.end());
  }

  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), points_num * point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  // Note: due to atomic operations in the kernel, generateVoxels does not handle overflows in the
  // counter, and instead is done in the following kernel
  cudaError_t code = autoware::lidar_centerpoint::generateVoxels_random_launch(
    points_d_.get(), points_num, range_min_x_, range_max_x_, range_min_y_, range_max_y_,
    range_min_z_, range_max_z_, voxel_size_x_, voxel_size_y_, voxel_size_z_, grid_size_y_,
    grid_size_x_, mask_d_.get(), voxels_buffer_d_.get(), stream_);

  ASSERT_EQ(cudaSuccess, code);

  // Check that the voxel was filled
  int voxel_idx = std::floor((points[0] - range_min_x_) / voxel_size_x_);
  int voxel_idy = std::floor((points[1] - range_min_y_) / voxel_size_y_);
  unsigned int voxel_index = voxel_idy * grid_size_x_ + voxel_idx;

  std::vector<float> voxel_data{};
  voxel_data.resize((max_point_in_voxel_size_ + 1) * point_feature_size_);

  CHECK_CUDA_ERROR(cudaMemcpy(
    voxel_data.data(),
    voxels_buffer_d_.get() + voxel_index * max_point_in_voxel_size_ * point_feature_size_,
    (max_point_in_voxel_size_ + 1) * point_feature_size_ * sizeof(unsigned int),
    cudaMemcpyDeviceToHost));

  for (std::size_t i = 0; i < max_point_in_voxel_size_; ++i) {
    EXPECT_EQ(points[0], voxel_data[i * point_feature_size_ + 0]);
    EXPECT_EQ(points[1], voxel_data[i * point_feature_size_ + 1]);
    EXPECT_EQ(points[2], voxel_data[i * point_feature_size_ + 2]);
    EXPECT_EQ(points[3], voxel_data[i * point_feature_size_ + 3]);
  }

  EXPECT_EQ(0.0, voxel_data[max_point_in_voxel_size_ * point_feature_size_ + 0]);
  EXPECT_EQ(0.0, voxel_data[max_point_in_voxel_size_ * point_feature_size_ + 1]);
  EXPECT_EQ(0.0, voxel_data[max_point_in_voxel_size_ * point_feature_size_ + 2]);
  EXPECT_EQ(0.0, voxel_data[max_point_in_voxel_size_ * point_feature_size_ + 3]);

  code = generateBaseFeatures_launch(
    mask_d_.get(), voxels_buffer_d_.get(), grid_size_y_, grid_size_x_, max_voxel_size_,
    num_voxels_d_.get(), voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(),
    stream_);

  ASSERT_EQ(cudaSuccess, code);

  unsigned int num_pillars{};
  std::vector<float> voxel_features(max_point_in_voxel_size_ * point_feature_size_, 0.f);
  float num_voxels{};
  std::vector<int> voxel_coordinates(3, 0);

  CHECK_CUDA_ERROR(cudaMemcpy(
    &num_pillars, num_voxels_d_.get(), 1 * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  CHECK_CUDA_ERROR(cudaMemcpy(
    voxel_features.data(), voxels_d_.get(),
    max_point_in_voxel_size_ * point_feature_size_ * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  CHECK_CUDA_ERROR(cudaMemcpy(
    &num_voxels, num_points_per_voxel_d_.get(), 1 * sizeof(float), cudaMemcpyDeviceToHost));

  CHECK_CUDA_ERROR(cudaMemcpy(
    voxel_coordinates.data(), coordinates_d_.get(), 3 * sizeof(int), cudaMemcpyDeviceToHost));

  EXPECT_EQ(1, num_pillars);
  EXPECT_EQ(static_cast<float>(max_point_in_voxel_size_), num_voxels);
  EXPECT_EQ(0, voxel_coordinates[0]);
  EXPECT_EQ(voxel_idy, voxel_coordinates[1]);
  EXPECT_EQ(voxel_idx, voxel_coordinates[2]);

  for (std::size_t point_index = 0; point_index < max_point_in_voxel_size_; ++point_index) {
    EXPECT_EQ(points[0], voxel_features[point_index * point_feature_size_ + 0]);
    EXPECT_EQ(points[1], voxel_features[point_index * point_feature_size_ + 1]);
    EXPECT_EQ(points[2], voxel_features[point_index * point_feature_size_ + 2]);
    EXPECT_EQ(points[3], voxel_features[point_index * point_feature_size_ + 3]);
  }

  code = generateFeatures_launch(
    voxels_d_.get(), num_points_per_voxel_d_.get(), coordinates_d_.get(), num_voxels_d_.get(),
    max_voxel_size_, voxel_size_x_, voxel_size_y_, voxel_size_z_, range_min_x_, range_min_y_,
    range_min_z_, encoder_in_features_d_.get(), stream_);

  ASSERT_EQ(cudaSuccess, code);

  std::vector<float> encoder_features(
    max_point_in_voxel_size_ * config_encoder_in_feature_size_, 0.f);

  CHECK_CUDA_ERROR(cudaMemcpy(
    encoder_features.data(), encoder_in_features_d_.get(),
    max_point_in_voxel_size_ * config_encoder_in_feature_size_ * sizeof(float),
    cudaMemcpyDeviceToHost));

  float voxel_x_offset = voxel_size_x_ / 2 + voxel_idx * voxel_size_x_ + range_min_x_;
  float voxel_y_offset = voxel_size_y_ / 2 + voxel_idy * voxel_size_y_ + range_min_y_;

  for (std::size_t point_index = 0; point_index < max_point_in_voxel_size_; ++point_index) {
    // The first four values are just the point features
    EXPECT_EQ(
      points[point_index * point_feature_size_ + 0],
      encoder_features[point_index * config_encoder_in_feature_size_ + 0]);
    EXPECT_EQ(
      points[point_index * point_feature_size_ + 1],
      encoder_features[point_index * config_encoder_in_feature_size_ + 1]);
    EXPECT_EQ(
      points[point_index * point_feature_size_ + 2],
      encoder_features[point_index * config_encoder_in_feature_size_ + 2]);
    EXPECT_EQ(
      points[point_index * point_feature_size_ + 3],
      encoder_features[point_index * config_encoder_in_feature_size_ + 3]);

    // The next three values are the relative coordinates with respect to the voxel average
    EXPECT_NEAR(0.0, encoder_features[point_index * config_encoder_in_feature_size_ + 4], 1e-4);
    EXPECT_NEAR(0.0, encoder_features[point_index * config_encoder_in_feature_size_ + 5], 1e-4);
    EXPECT_NEAR(0.0, encoder_features[point_index * config_encoder_in_feature_size_ + 6], 1e-4);

    // The last two values are the relative coordinates with respect to the voxel center
    EXPECT_EQ(
      points[point_index * point_feature_size_ + 0] - voxel_x_offset,
      encoder_features[point_index * config_encoder_in_feature_size_ + 7]);
    EXPECT_EQ(
      points[point_index * point_feature_size_ + 1] - voxel_y_offset,
      encoder_features[point_index * config_encoder_in_feature_size_ + 8]);
  }
}

}  // namespace autoware::lidar_centerpoint

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
