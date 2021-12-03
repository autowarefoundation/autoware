// Copyright 2021 Tier IV, Inc.
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

#ifndef HEATMAP_UTILS_HPP_
#define HEATMAP_UTILS_HPP_

#include <torch/script.h>

#include <tuple>

namespace centerpoint
{
at::Tensor sigmoid_hm(const at::Tensor & heatmap)
{
  // heatmap (float): (batch_size, num_class, H, W)

  return torch::clamp(torch::sigmoid(heatmap), /*min=*/1e-6, /*max=*/1 - 1e-6);
}

at::Tensor nms_hm(const at::Tensor & heatmap, const int kernel_size = 3)
{
  // heatmap (float): (B, C, H, W)

  at::Tensor heatmap_max = torch::max_pool2d(
    heatmap, {kernel_size, kernel_size},
    /*stride=*/{1}, /*padding=*/{(kernel_size - 1) / 2});
  at::Tensor mask = heatmap_max == heatmap;
  return heatmap * mask.to(heatmap.dtype());
}

at::Tensor gather_feature(const at::Tensor & feature, const at::Tensor & index)
{
  // feature (float): (batch_size, topk * num_class, 1)
  // feature (int): (batch_size, topk)

  int channel = feature.sizes()[2];
  auto index_size = index.sizes();
  at::Tensor _index = index.unsqueeze(-1).expand({index_size[0], index_size[1], channel});
  at::Tensor _feature = feature.gather(1, _index);
  return _feature;
}

std::tuple<at::Tensor, at::Tensor, at::Tensor, at::Tensor, at::Tensor> select_topk(
  const at::Tensor & heatmap_pred, const int k)
{
  // heatmap_pred: (batch_size, num_class, H, W)

  const auto dtype = heatmap_pred.dtype();
  const int batch_size = heatmap_pred.sizes()[0];
  const int cls = heatmap_pred.sizes()[1];
  const int width = heatmap_pred.sizes()[3];

  // first select topk scores in all classes and batches
  // [B, C, H, W] -> [B, C, H*W]
  at::Tensor _heatmap_pred = heatmap_pred.view({batch_size, cls, -1});

  // both in [B, C, K]
  auto topk_all_tuple = _heatmap_pred.topk(k);
  at::Tensor topk_scores_all = std::get<0>(topk_all_tuple);
  at::Tensor topk_inds_all = std::get<1>(topk_all_tuple);

  at::Tensor topk_ys = topk_inds_all.to(dtype).floor_divide(width);
  at::Tensor topk_xs = topk_inds_all.to(dtype).fmod(width);

  // select topk examples across channels
  // [B, C, K] -> [B, C*K]
  topk_scores_all = topk_scores_all.view({batch_size, -1});

  // Both in [N, K]
  auto topk_tuple = topk_scores_all.topk(k);
  at::Tensor topk_scores = std::get<0>(topk_tuple);
  at::Tensor topk_inds = std::get<1>(topk_tuple);
  at::Tensor topk_clses = topk_inds.to(dtype).floor_divide(k);

  topk_inds_all = topk_inds_all.view({batch_size, -1, 1});
  topk_ys = topk_ys.view({batch_size, -1, 1});
  topk_xs = topk_xs.view({batch_size, -1, 1});

  topk_inds_all = gather_feature(topk_inds_all, topk_inds).view({batch_size, k});
  topk_ys = gather_feature(topk_ys, topk_inds).view({batch_size, k});
  topk_xs = gather_feature(topk_xs, topk_inds).view({batch_size, k});

  return std::make_tuple(topk_scores, topk_inds_all, topk_clses, topk_ys, topk_xs);
}

at::Tensor select_point_of_interest(const at::Tensor & index, const at::Tensor & feature_map)
{
  // index: (batch_size, N)
  // feature_map: (batch_size, num_features, H, W)

  const int batch_size = feature_map.sizes()[0];
  const int channel = feature_map.sizes()[1];
  at::Tensor _index = index.view({batch_size, -1, 1}).repeat({1, 1, channel});
  at::Tensor _feature_map = feature_map.permute({0, 2, 3, 1}).contiguous();
  _feature_map = _feature_map.view({batch_size, -1, channel});
  _feature_map = _feature_map.gather(1, _index);
  return _feature_map;
}

}  // namespace centerpoint

#endif  // HEATMAP_UTILS_HPP_
