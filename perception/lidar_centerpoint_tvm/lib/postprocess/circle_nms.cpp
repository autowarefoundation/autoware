// Copyright 2022 AutoCore Ltd.
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

#include "lidar_centerpoint_tvm/postprocess/circle_nms.hpp"

#include <lidar_centerpoint_tvm/utils.hpp>

#include <cmath>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{

inline float dist2dPow(const Box3D & a, const Box3D & b)
{
  return powf(a.x - b.x, 2) + powf(a.y - b.y, 2);
}

std::size_t circleNMS(
  std::vector<Box3D> & boxes3d, const float dist_thresh, std::vector<bool> & keep_mask)
{
  // params: boxes3d, vector sorted by score from largest to smallest
  const auto num_boxes3d = boxes3d.size();
  const float dist2d_pow_thresh = powf(dist_thresh, 2);

  // generate keep_mask
  std::size_t num_to_keep = 0;
  std::vector<bool> suppress(num_boxes3d);  // suppress[i]=true mean i-th box should be suppressed

  // std::uint64_t * suppress_ptr = & suppress.front();
  for (std::size_t i = 0; i < num_boxes3d; i++) {
    if (suppress[i]) {
      keep_mask[i] = false;
    } else {
      keep_mask[i] = true;
      num_to_keep++;
      for (std::size_t j = i + 1; j < num_boxes3d; j++) {
        // if the j-th box has been suppressed, continue
        if (suppress[j]) continue;
        // if the j-th box is in the circle of i-th, set j-th box to be suppressed
        if (dist2dPow(boxes3d[i], boxes3d[j]) < dist2d_pow_thresh) suppress[j] = true;
      }
    }
  }

  return num_to_keep;
}

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware
