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

#include "cuda_utils.hpp"
#include "gather_topk_kernel.hpp"
#include "trt_plugin_helper.hpp"

#include <functional>
#include <numeric>
#include <vector>

namespace ssd
{
template <typename scalar_t>
__global__ void gather_topk_kernel(
  const scalar_t * input, const int * indices, scalar_t * output, int batch, int num_input,
  int num_indices, int channel)
{
  CUDA_1D_KERNEL_LOOP(index, batch * num_indices * channel)
  {
    const int b_id = index / (num_indices * channel);
    const int n_id = (index / channel) % num_indices;
    const int c_id = index % channel;

    const int input_n_id = indices[b_id * num_indices + n_id];
    const scalar_t value = input[b_id * num_input * channel + input_n_id * channel + c_id];
    output[b_id * num_indices * channel + n_id * channel + c_id] = value;
  }
}

template <typename scalar_t>
void gather_topk_impl(
  const scalar_t * input, const int * indices, const int * dims, int nbDims,
  const int * dims_indices, int nbDims_index, scalar_t * output, cudaStream_t stream)
{
  int batch = 1;
  for (int i = 0; i < nbDims_index - 1; ++i) {
    batch *= dims[i];
  }
  int num_input = dims[nbDims_index - 1];
  int num_indices = dims_indices[nbDims_index - 1];
  int channel = 1;
  for (int i = nbDims_index; i < nbDims; ++i) {
    channel *= dims[i];
  }
  const int col_block = DIVUP(batch * num_indices * channel, THREADS_PER_BLOCK);
  gather_topk_kernel<<<col_block, THREADS_PER_BLOCK, 0, stream>>>(
    input, indices, output, batch, num_input, num_indices, channel);
}

template void gather_topk_impl<float>(
  const float * input, const int * indices, const int * dims, int nbDims, const int * indices_dims,
  int indices_nbDims, float * output, cudaStream_t stream);

template void gather_topk_impl<int32_t>(
  const int32_t * input, const int * indices, const int * dims, int nbDims,
  const int * indices_dims, int indices_nbDims, int32_t * output, cudaStream_t stream);
}  // namespace ssd
