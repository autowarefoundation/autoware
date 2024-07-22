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

#include <autoware/lidar_transfusion/cuda_utils.hpp>
#include <autoware/lidar_transfusion/postprocess/postprocess_kernel.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::lidar_transfusion
{

class PostprocessKernelTest : public testing::Test
{
public:
  void SetUp() override;
  void TearDown() override;

  std::unique_ptr<PostprocessCuda> post_ptr_{nullptr};
  std::unique_ptr<TransfusionConfig> config_ptr_{};
  cudaStream_t stream_{};

  unsigned int cls_size_{0};
  unsigned int box_size_{0};
  unsigned int dir_cls_size_{0};
  cuda::unique_ptr<float[]> cls_output_d_{nullptr};
  cuda::unique_ptr<float[]> box_output_d_{nullptr};
  cuda::unique_ptr<float[]> dir_cls_output_d_{nullptr};
};

}  // namespace autoware::lidar_transfusion

#endif  // TEST_POSTPROCESS_KERNEL_HPP_
