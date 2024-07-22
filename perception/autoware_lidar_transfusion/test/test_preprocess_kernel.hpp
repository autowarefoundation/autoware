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

#ifndef TEST_PREPROCESS_KERNEL_HPP_
#define TEST_PREPROCESS_KERNEL_HPP_

#include <autoware/lidar_transfusion/cuda_utils.hpp>
#include <autoware/lidar_transfusion/preprocess/preprocess_kernel.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::lidar_transfusion
{

class PreprocessKernelTest : public testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

  std::unique_ptr<PreprocessCuda> pre_ptr_{nullptr};
  std::unique_ptr<TransfusionConfig> config_ptr_{};
  cudaStream_t stream_{};

  unsigned int voxel_features_size_{0};
  unsigned int voxel_num_size_{0};
  unsigned int voxel_idxs_size_{0};
  cuda::unique_ptr<float[]> points_d_{nullptr};
  cuda::unique_ptr<unsigned int> params_input_d_{nullptr};
  cuda::unique_ptr<float[]> voxel_features_d_{nullptr};
  cuda::unique_ptr<unsigned int[]> voxel_num_d_{nullptr};
  cuda::unique_ptr<unsigned int[]> voxel_idxs_d_{nullptr};
};

}  // namespace autoware::lidar_transfusion

#endif  // TEST_PREPROCESS_KERNEL_HPP_
