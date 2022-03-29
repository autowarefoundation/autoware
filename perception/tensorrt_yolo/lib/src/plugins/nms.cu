/// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <cuda.h>
#include <thrust/device_ptr.h>
#include <thrust/execution_policy.h>
#include <thrust/gather.h>
#include <thrust/sequence.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

#if CUDART_VERSION >= 11000
#include <cub/device/device_radix_sort.cuh>
#include <cub/iterator/counting_input_iterator.cuh>
#else
#include <thrust/system/cuda/detail/cub/device/device_radix_sort.cuh>
#include <thrust/system/cuda/detail/cub/iterator/counting_input_iterator.cuh>
using namespace thrust::cuda_cub;
#endif

#include <cuda_utils.hpp>
#include <nms.hpp>

namespace yolo
{
__global__ void nms_kernel(
  const int num_per_thread, const float threshold, const int num_detections, const int * indices,
  float * scores, const float * classes, const float4 * boxes)
{
  // Go through detections by descending score
  for (int m = 0; m < num_detections; m++) {
    for (int n = 0; n < num_per_thread; n++) {
      int i = threadIdx.x * num_per_thread + n;
      if (i < num_detections && m < i && scores[m] > 0.0f) {
        int idx = indices[i];
        int max_idx = indices[m];
        int icls = classes[idx];
        int mcls = classes[max_idx];
        if (mcls == icls) {
          float4 ibox = boxes[idx];
          float4 mbox = boxes[max_idx];
          float x1 = max(ibox.x, mbox.x);
          float y1 = max(ibox.y, mbox.y);
          float x2 = min(ibox.x + ibox.z, mbox.x + mbox.z);
          float y2 = min(ibox.y + ibox.w, ibox.y + mbox.w);
          float w = max(0.0f, x2 - x1);
          float h = max(0.0f, y2 - y1);
          float iarea = ibox.z * ibox.w;
          float marea = mbox.z * mbox.w;
          float inter = w * h;
          float overlap = inter / (iarea + marea - inter);
          if (overlap > threshold) {
            scores[i] = 0.0f;
          }
        }
      }
    }

    // Sync discarded detections
    __syncthreads();
  }
}

int nms(
  int batch_size, const void * const * inputs, void * const * outputs, size_t count,
  int detections_per_im, float nms_thresh, void * workspace, size_t workspace_size,
  cudaStream_t stream)
{
  if (!workspace || !workspace_size) {
    // Return required scratch space size cub style
    workspace_size = cuda::get_size_aligned<bool>(count);    // flags
    workspace_size += cuda::get_size_aligned<int>(count);    // indices
    workspace_size += cuda::get_size_aligned<int>(count);    // indices_sorted
    workspace_size += cuda::get_size_aligned<float>(count);  // scores
    workspace_size += cuda::get_size_aligned<float>(count);  // scores_sorted

    size_t temp_size_flag = 0;
    cub::DeviceSelect::Flagged(
      (void *)nullptr, temp_size_flag, cub::CountingInputIterator<int>(count), (bool *)nullptr,
      (int *)nullptr, (int *)nullptr, count);
    size_t temp_size_sort = 0;
    cub::DeviceRadixSort::SortPairsDescending(
      (void *)nullptr, temp_size_sort, (float *)nullptr, (float *)nullptr, (int *)nullptr,
      (int *)nullptr, count);
    workspace_size += std::max(temp_size_flag, temp_size_sort);

    return workspace_size;
  }

  auto on_stream = thrust::cuda::par.on(stream);

  auto flags = cuda::get_next_ptr<bool>(count, workspace, workspace_size);
  auto indices = cuda::get_next_ptr<int>(count, workspace, workspace_size);
  auto indices_sorted = cuda::get_next_ptr<int>(count, workspace, workspace_size);
  auto scores = cuda::get_next_ptr<float>(count, workspace, workspace_size);
  auto scores_sorted = cuda::get_next_ptr<float>(count, workspace, workspace_size);

  for (int batch = 0; batch < batch_size; batch++) {
    auto in_scores = static_cast<const float *>(inputs[0]) + batch * count;
    auto in_boxes = static_cast<const float4 *>(inputs[1]) + batch * count;
    auto in_classes = static_cast<const float *>(inputs[2]) + batch * count;

    auto out_scores = static_cast<float *>(outputs[0]) + batch * detections_per_im;
    auto out_boxes = static_cast<float4 *>(outputs[1]) + batch * detections_per_im;
    auto out_classes = static_cast<float *>(outputs[2]) + batch * detections_per_im;

    // Discard null scores
    thrust::transform(
      on_stream, in_scores, in_scores + count, flags, thrust::placeholders::_1 > 0.0f);

    int * num_selected = reinterpret_cast<int *>(indices_sorted);
    cub::DeviceSelect::Flagged(
      workspace, workspace_size, cub::CountingInputIterator<int>(0), flags, indices, num_selected,
      count, stream);
    cudaStreamSynchronize(stream);
    int num_detections = *thrust::device_pointer_cast(num_selected);

    // Sort scores and corresponding indices
    thrust::gather(on_stream, indices, indices + num_detections, in_scores, scores);
    cub::DeviceRadixSort::SortPairsDescending(
      workspace, workspace_size, scores, scores_sorted, indices, indices_sorted, num_detections, 0,
      sizeof(*scores) * 8, stream);

    // Launch actual NMS kernel - 1 block with each thread handling n detections
    const int max_threads = 1024;
    int num_per_thread = ceil((float)num_detections / max_threads);
    nms_kernel<<<1, max_threads, 0, stream>>>(
      num_per_thread, nms_thresh, num_detections, indices_sorted, scores_sorted, in_classes,
      in_boxes);

    // Re-sort with updated scores
    cub::DeviceRadixSort::SortPairsDescending(
      workspace, workspace_size, scores_sorted, scores, indices_sorted, indices, num_detections, 0,
      sizeof(*scores) * 8, stream);

    // Gather filtered scores, boxes, classes
    num_detections = min(detections_per_im, num_detections);
    cudaMemcpyAsync(
      out_scores, scores, num_detections * sizeof *scores, cudaMemcpyDeviceToDevice, stream);
    if (num_detections < detections_per_im) {
      thrust::fill_n(on_stream, out_scores + num_detections, detections_per_im - num_detections, 0);
    }
    thrust::gather(on_stream, indices, indices + num_detections, in_boxes, out_boxes);
    thrust::gather(on_stream, indices, indices + num_detections, in_classes, out_classes);
  }

  return 0;
}

}  // namespace yolo
