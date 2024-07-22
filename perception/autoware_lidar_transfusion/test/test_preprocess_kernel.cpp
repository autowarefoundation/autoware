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

#include "autoware/lidar_transfusion/cuda_utils.hpp"

#include <autoware/lidar_transfusion/preprocess/preprocess_kernel.hpp>

#include <gtest/gtest.h>
#include <thrust/host_vector.h>
#include <thrust/reduce.h>

#include <cmath>
#include <vector>

namespace autoware::lidar_transfusion
{

void PreprocessKernelTest::SetUp()
{
  cudaStreamCreate(&stream_);

  auto cloud_capacity = 2000000;
  auto voxels_num = std::vector<int64_t>{1, 3, 5};
  auto point_cloud_range = std::vector<double>{-76.8, -76.8, -3.0, 76.8, 76.8, 5.0};
  auto voxel_size = std::vector<double>{0.3, 0.3, 8.0};
  auto num_proposals = 500;
  auto score_threshold = 0.2f;
  auto circle_nms_dist_threshold = 0.5f;
  auto yaw_norm_thresholds = std::vector<double>{0.5, 0.5, 0.5};
  config_ptr_ = std::make_unique<TransfusionConfig>(
    cloud_capacity, voxels_num, point_cloud_range, voxel_size, num_proposals,
    circle_nms_dist_threshold, yaw_norm_thresholds, score_threshold);
  pre_ptr_ = std::make_unique<PreprocessCuda>(*config_ptr_, stream_);

  voxel_features_size_ = config_ptr_->max_voxels_ * config_ptr_->max_num_points_per_pillar_ *
                         config_ptr_->num_point_feature_size_;
  voxel_num_size_ = config_ptr_->max_voxels_;
  voxel_idxs_size_ = config_ptr_->max_voxels_ * config_ptr_->num_point_values_;

  params_input_d_ = cuda::make_unique<unsigned int>();
  voxel_features_d_ = cuda::make_unique<float[]>(voxel_features_size_);
  voxel_num_d_ = cuda::make_unique<unsigned int[]>(voxel_num_size_);
  voxel_idxs_d_ = cuda::make_unique<unsigned int[]>(voxel_idxs_size_);
  points_d_ =
    cuda::make_unique<float[]>(config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_);

  cuda::clear_async(voxel_features_d_.get(), voxel_features_size_, stream_);
  cuda::clear_async(voxel_num_d_.get(), voxel_num_size_, stream_);
  cuda::clear_async(voxel_idxs_d_.get(), voxel_idxs_size_, stream_);
  cuda::clear_async(params_input_d_.get(), 1, stream_);
  cuda::clear_async(
    points_d_.get(), config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

void PreprocessKernelTest::TearDown()
{
}

TEST_F(PreprocessKernelTest, EmptyVoxelTest)
{
  std::vector<float> points{};
  std::size_t count = 0;

  pre_ptr_->generateVoxels(
    points_d_.get(), count, params_input_d_.get(), voxel_features_d_.get(), voxel_num_d_.get(),
    voxel_idxs_d_.get());
  unsigned int params_input;
  auto code1 = cudaStreamSynchronize(stream_);

  auto code2 = cudaMemcpyAsync(
    &params_input, params_input_d_.get(), sizeof(unsigned int), cudaMemcpyDeviceToHost, stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  auto code3 = cudaMemcpy(
    points_d_.get(), points.data(), count * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice);

  ASSERT_EQ(cudaSuccess, code1);
  ASSERT_EQ(cudaSuccess, code2);
  ASSERT_EQ(cudaSuccess, code3);

  EXPECT_EQ(0, params_input);
}

TEST_F(PreprocessKernelTest, BasicTest)
{
  cuda::clear_async(voxel_features_d_.get(), voxel_features_size_, stream_);
  cuda::clear_async(voxel_num_d_.get(), voxel_num_size_, stream_);
  cuda::clear_async(voxel_idxs_d_.get(), voxel_idxs_size_, stream_);
  cuda::clear_async(params_input_d_.get(), 1, stream_);
  cuda::clear_async(
    points_d_.get(), config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_, stream_);
  cudaStreamSynchronize(stream_);

  std::vector<float> point1{25.f, -61.1f, 1.8f, 42.0, 0.1f};
  std::vector<float> point2{-31.6f, 1.1f, 2.8f, 77.0, 0.1f};
  std::vector<float> point3{42.6f, 15.1f, -0.1f, 3.0, 0.1f};

  std::vector<float> points;
  points.reserve(point1.size() + point2.size() + point3.size());
  points.insert(points.end(), point1.begin(), point1.end());
  points.insert(points.end(), point2.begin(), point2.end());
  points.insert(points.end(), point3.begin(), point3.end());
  std::size_t count = 3;

  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), count * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  // Compute the total amount of voxels filled
  pre_ptr_->generateVoxels(
    points_d_.get(), count, params_input_d_.get(), voxel_features_d_.get(), voxel_num_d_.get(),
    voxel_idxs_d_.get());

  unsigned int params_input;
  CHECK_CUDA_ERROR(
    cudaMemcpy(&params_input, params_input_d_.get(), sizeof(unsigned int), cudaMemcpyDeviceToHost));
  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), count * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  ASSERT_EQ(count, params_input);

  // Check if voxels were filled
  unsigned int voxel_num;

  for (std::size_t i = 0; i < count; ++i) {
    CHECK_CUDA_ERROR(
      cudaMemcpy(&voxel_num, voxel_num_d_.get() + i, sizeof(unsigned int), cudaMemcpyDeviceToHost));
    EXPECT_EQ(1, voxel_num);
  }

  // Check grid indices
  thrust::host_vector<unsigned int> voxel_idxs(config_ptr_->num_point_values_ * count, 0);
  unsigned int voxel_idx_gt;
  unsigned int voxel_idy_gt;
  CHECK_CUDA_ERROR(cudaMemcpy(
    voxel_idxs.data(), voxel_idxs_d_.get(),
    config_ptr_->num_point_values_ * count * sizeof(unsigned int), cudaMemcpyDeviceToHost));

  for (std::size_t i = 0; i < count; ++i) {
    voxel_idx_gt = std::floor(
      (points[config_ptr_->num_point_feature_size_ * i + 0] - config_ptr_->min_x_range_) /
      config_ptr_->voxel_x_size_);
    voxel_idy_gt = std::floor(
      (points[config_ptr_->num_point_feature_size_ * i + 1] - config_ptr_->min_y_range_) /
      config_ptr_->voxel_y_size_);

    EXPECT_EQ(
      voxel_idx_gt,
      voxel_idxs[config_ptr_->num_point_values_ * i + 3]);  // {0, 0, voxel_idy, VOXEL_IDX}
    EXPECT_EQ(
      voxel_idy_gt,
      voxel_idxs[config_ptr_->num_point_values_ * i + 2]);  // {0, 0, VOXEL_IDY, voxel_idx}
  }

  // Check voxel features
  thrust::host_vector<float> voxel_features(
    count * config_ptr_->max_num_points_per_pillar_ * config_ptr_->num_point_feature_size_, 0.f);
  CHECK_CUDA_ERROR(cudaMemcpy(
    voxel_features.data(), voxel_features_d_.get(),
    count * config_ptr_->max_num_points_per_pillar_ * config_ptr_->num_point_feature_size_ *
      sizeof(float),
    cudaMemcpyDeviceToHost));

  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = 0; j < config_ptr_->num_point_feature_size_; ++j) {
      EXPECT_EQ(
        points[config_ptr_->num_point_feature_size_ * i + j],
        voxel_features
          [i * config_ptr_->max_num_points_per_pillar_ * config_ptr_->num_point_feature_size_ + j]);
    }
  }
}

TEST_F(PreprocessKernelTest, OutOfRangeTest)
{
  cuda::clear_async(voxel_features_d_.get(), voxel_features_size_, stream_);
  cuda::clear_async(voxel_num_d_.get(), voxel_num_size_, stream_);
  cuda::clear_async(voxel_idxs_d_.get(), voxel_idxs_size_, stream_);
  cuda::clear_async(params_input_d_.get(), 1, stream_);
  cuda::clear_async(
    points_d_.get(), config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_, stream_);
  cudaStreamSynchronize(stream_);

  std::vector<float> points{25.f, config_ptr_->max_y_range_ + 0.1f, 2.1f, 55.f, 0.1f};
  std::size_t count = 0;

  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), count * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  pre_ptr_->generateVoxels(
    points_d_.get(), count, params_input_d_.get(), voxel_features_d_.get(), voxel_num_d_.get(),
    voxel_idxs_d_.get());

  unsigned int params_input;
  CHECK_CUDA_ERROR(
    cudaMemcpy(&params_input, params_input_d_.get(), sizeof(unsigned int), cudaMemcpyDeviceToHost));
  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), count * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  EXPECT_EQ(count, params_input);
}

TEST_F(PreprocessKernelTest, VoxelOverflowTest)
{
  cuda::clear_async(voxel_features_d_.get(), voxel_features_size_, stream_);
  cuda::clear_async(voxel_num_d_.get(), voxel_num_size_, stream_);
  cuda::clear_async(voxel_idxs_d_.get(), voxel_idxs_size_, stream_);
  cuda::clear_async(params_input_d_.get(), 1, stream_);
  cuda::clear_async(
    points_d_.get(), config_ptr_->cloud_capacity_ * config_ptr_->num_point_feature_size_, stream_);
  cudaStreamSynchronize(stream_);

  std::vector<float> point{config_ptr_->min_x_range_, -61.1f, 1.8f, 42.f, 0.1f};
  std::size_t count = config_ptr_->max_voxels_ + 1;
  std::vector<float> points{};
  points.reserve(count * config_ptr_->num_point_feature_size_);

  for (std::size_t i = 0; i < count; ++i) {
    points.insert(points.end(), point.begin(), point.end());
    point[0] += config_ptr_->voxel_x_size_;
  }

  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), count * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  pre_ptr_->generateVoxels(
    points_d_.get(), count, params_input_d_.get(), voxel_features_d_.get(), voxel_num_d_.get(),
    voxel_idxs_d_.get());

  ASSERT_EQ(cudaSuccess, cudaGetLastError());

  unsigned int params_input;
  CHECK_CUDA_ERROR(
    cudaMemcpy(&params_input, params_input_d_.get(), sizeof(unsigned int), cudaMemcpyDeviceToHost));
  CHECK_CUDA_ERROR(cudaMemcpy(
    points_d_.get(), points.data(), count * config_ptr_->num_point_feature_size_ * sizeof(float),
    cudaMemcpyHostToDevice));

  EXPECT_EQ(config_ptr_->max_voxels_, params_input);
}

}  // namespace autoware::lidar_transfusion

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
