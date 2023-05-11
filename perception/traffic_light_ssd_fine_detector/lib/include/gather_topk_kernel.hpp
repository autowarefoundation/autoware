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

#ifndef GATHER_TOPK_KERNEL_HPP_
#define GATHER_TOPK_KERNEL_HPP_

#include <cuda_runtime.h>

namespace ssd
{
template <typename scalar_t>
void gather_topk_impl(
  const scalar_t * input, const int * indices, const int * dims, int nbDims,
  const int * indices_dims, int indices_nbDims, scalar_t * output, cudaStream_t stream);
}  // namespace ssd

#endif  // GATHER_TOPK_KERNEL_HPP_
