// Copyright 2023 TIER IV, Inc.
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

// Copyright (c) OpenMMLab. All rights reserved.

#ifndef GRID_PRIORS_KERNEL_HPP_
#define GRID_PRIORS_KERNEL_HPP_

#include <cuda_runtime.h>

namespace ssd
{
template <typename scalar_t>
void grid_priors_impl(
  const scalar_t * base_anchor, scalar_t * output, int num_base_anchors, int feat_w, int feat_h,
  int stride_w, int stride_h, cudaStream_t stream);
}  // namespace ssd

#endif  // GRID_PRIORS_KERNEL_HPP_
