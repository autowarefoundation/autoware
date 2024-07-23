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

#ifndef TEST_POSTPROCESS_KERNEL_HPP_
#define TEST_POSTPROCESS_KERNEL_HPP_

#include <autoware/lidar_centerpoint/cuda_utils.hpp>
#include <autoware/lidar_centerpoint/postprocess/postprocess_kernel.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::lidar_centerpoint
{

class PostprocessKernelTest : public testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

  cudaStream_t stream_{};

  std::unique_ptr<PostProcessCUDA> postprocess_cuda_ptr_{};
  std::unique_ptr<autoware::lidar_centerpoint::CenterPointConfig> config_ptr_{};

  cuda::unique_ptr<float[]> head_out_heatmap_d_{};
  cuda::unique_ptr<float[]> head_out_offset_d_{};
  cuda::unique_ptr<float[]> head_out_z_d_{};
  cuda::unique_ptr<float[]> head_out_dim_d_{};
  cuda::unique_ptr<float[]> head_out_rot_d_{};
  cuda::unique_ptr<float[]> head_out_vel_d_{};
};

}  // namespace autoware::lidar_centerpoint

#endif  // TEST_POSTPROCESS_KERNEL_HPP_
