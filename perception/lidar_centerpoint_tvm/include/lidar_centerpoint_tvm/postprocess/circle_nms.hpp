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

#ifndef LIDAR_CENTERPOINT_TVM__POSTPROCESS__CIRCLE_NMS_HPP_
#define LIDAR_CENTERPOINT_TVM__POSTPROCESS__CIRCLE_NMS_HPP_

#include <lidar_centerpoint_tvm/utils.hpp>

#include <vector>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{
// Non-maximum suppression (NMS) uses the distance on the xy plane instead of
// intersection over union (IoU) to suppress overlapped objects.
std::size_t circleNMS(
  std::vector<Box3D> & boxes3d, const float dist_thresh, std::vector<bool> & keep_mask);

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_CENTERPOINT_TVM__POSTPROCESS__CIRCLE_NMS_HPP_
