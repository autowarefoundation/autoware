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

#ifndef AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__POSTPROCESS_KERNEL_HPP_

#include "autoware/lidar_transfusion/transfusion_config.hpp"
#include "autoware/lidar_transfusion/utils.hpp"

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <vector>

namespace autoware::lidar_transfusion
{

class PostprocessCuda
{
public:
  explicit PostprocessCuda(const TransfusionConfig & config, cudaStream_t & stream);

  cudaError_t generateDetectedBoxes3D_launch(
    const float * cls_output, const float * box_output, const float * dir_cls_output,
    std::vector<Box3D> & det_boxes3d, cudaStream_t stream);

private:
  TransfusionConfig config_;
  cudaStream_t stream_;
  cudaStream_t stream_event_;
  cudaEvent_t start_, stop_;
};

}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
