// Copyright 2020 Tier IV, Inc.
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
 * MIT License

 * Copyright (c) 2019-2020 Wang Xinyu

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

#include <cuda_utils.hpp>
#include <yolo_layer.hpp>

#include <math_constants.h>
#include <stdio.h>

#include <stdexcept>

namespace yolo
{
inline __device__ float sigmoid(float x) { return 1.0f / (1.0f + __expf(-x)); }

inline __device__ float scaleSigmoid(float x, float scale)
{
  return scale * sigmoid(x) - (scale - 1.0f) * 0.5f;
}

template <unsigned TPB>
__global__ void yoloLayerKernel(
  const float * input, float * out_scores, float4 * out_boxes, float * out_classes, int grid_width,
  int grid_height, int num_classes, int num_anchors, const float * anchors, int input_width,
  int input_height, float scale_x_y, float score_thresh, bool use_darknet_layer)
{
  int idx = threadIdx.x + TPB * blockIdx.x;
  int total_grids = grid_width * grid_height;
  if (idx >= total_grids * num_anchors) return;
  auto out_score = (out_scores) + idx;
  auto out_box = (out_boxes) + idx;
  auto out_class = (out_classes) + idx;

  int anchor_idx = idx / total_grids;
  idx = idx - total_grids * anchor_idx;
  int info_len = 5 + num_classes;
  auto cur_input = static_cast<const float *>(input) + anchor_idx * (info_len * total_grids);

  int class_id;
  float max_cls_logit = -CUDART_INF_F;  // minus infinity
  for (int i = 5; i < info_len; ++i) {
    float l = cur_input[idx + i * total_grids];
    if (l > max_cls_logit) {
      max_cls_logit = l;
      class_id = i - 5;
    }
  }
  float max_cls_prob = sigmoid(max_cls_logit);
  float objectness = sigmoid(cur_input[idx + 4 * total_grids]);

  int row = idx / grid_width;
  int col = idx % grid_width;
  float x = 0, y = 0, w = 0, h = 0;

  if (use_darknet_layer) {
    x = (col + scaleSigmoid(cur_input[idx + 0 * total_grids], scale_x_y)) / grid_width;    // [0, 1]
    y = (row + scaleSigmoid(cur_input[idx + 1 * total_grids], scale_x_y)) / grid_height;   // [0, 1]
    w = __expf(cur_input[idx + 2 * total_grids]) * anchors[2 * anchor_idx] / input_width;  // [0, 1]
    h = __expf(cur_input[idx + 3 * total_grids]) * anchors[2 * anchor_idx + 1] /
        input_height;  // [0, 1]
  } else {
    x = (col + sigmoid(cur_input[idx + 0 * total_grids]) * 2 - 0.5) / grid_width;   // [0, 1]
    y = (row + sigmoid(cur_input[idx + 1 * total_grids]) * 2 - 0.5) / grid_height;  // [0, 1]
    w = (sigmoid(cur_input[idx + 2 * total_grids]) * 2) *
        (sigmoid(cur_input[idx + 2 * total_grids]) * 2) * anchors[2 * anchor_idx] /
        input_width;  // [0, 1]
    h = (sigmoid(cur_input[idx + 3 * total_grids]) * 2) *
        (sigmoid(cur_input[idx + 3 * total_grids]) * 2) * anchors[2 * anchor_idx + 1] /
        input_height;  // [0, 1]
  }
  x -= w / 2;  // shift from center to top-left
  y -= h / 2;
  *out_box = make_float4(x, y, w, h);
  *out_class = class_id;
  *out_score = objectness < score_thresh ? 0.0 : max_cls_prob * objectness;
}

int yoloLayer(
  int batch_size, const void * const * inputs, void * const * outputs, int grid_width,
  int grid_height, int num_classes, int num_anchors, const std::vector<float> & anchors,
  int input_width, int input_height, float scale_x_y, float score_thresh, bool use_darknet_layer,
  void * workspace, size_t workspace_size, cudaStream_t stream)
{
  if (!workspace || !workspace_size) {
    workspace_size = cuda::get_size_aligned<float>(anchors.size());
    return workspace_size;
  }

  auto anchors_d = cuda::get_next_ptr<float>(anchors.size(), workspace, workspace_size);
  cudaMemcpyAsync(
    anchors_d, anchors.data(), anchors.size() * sizeof *anchors_d, cudaMemcpyHostToDevice, stream);

  int num_elements = num_anchors * grid_width * grid_height;
  constexpr int block_size = 256;
  const int grid_size = (num_elements + block_size - 1) / block_size;
  for (int batch = 0; batch < batch_size; ++batch) {
    auto input = static_cast<const float *>(inputs[0]) +
                 batch * num_anchors * (num_classes + 5) * grid_width * grid_height;
    auto out_scores = static_cast<float *>(outputs[0]) + batch * num_elements;
    auto out_boxes = static_cast<float4 *>(outputs[1]) + batch * num_elements;
    auto out_classes = static_cast<float *>(outputs[2]) + batch * num_elements;
    yoloLayerKernel<block_size><<<grid_size, block_size, 0, stream>>>(
      input, out_scores, out_boxes, out_classes, grid_width, grid_height, num_classes, num_anchors,
      anchors_d, input_width, input_height, scale_x_y, score_thresh, use_darknet_layer);
  }
  return 0;
}

}  // namespace yolo
