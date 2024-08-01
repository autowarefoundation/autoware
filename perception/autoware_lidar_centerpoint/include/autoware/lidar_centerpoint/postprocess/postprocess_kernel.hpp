// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_CENTERPOINT__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
#define AUTOWARE__LIDAR_CENTERPOINT__POSTPROCESS__POSTPROCESS_KERNEL_HPP_

#include "autoware/lidar_centerpoint/centerpoint_config.hpp"
#include "autoware/lidar_centerpoint/utils.hpp"
#include "cuda.h"
#include "cuda_runtime_api.h"

#include <vector>

namespace autoware::lidar_centerpoint
{
class PostProcessCUDA
{
public:
  explicit PostProcessCUDA(const CenterPointConfig & config);

  cudaError_t generateDetectedBoxes3D_launch(
    const float * out_heatmap, const float * out_offset, const float * out_z, const float * out_dim,
    const float * out_rot, const float * out_vel, std::vector<Box3D> & det_boxes3d,
    cudaStream_t stream);

private:
  CenterPointConfig config_;
};

}  // namespace autoware::lidar_centerpoint

#endif  // AUTOWARE__LIDAR_CENTERPOINT__POSTPROCESS__POSTPROCESS_KERNEL_HPP_
