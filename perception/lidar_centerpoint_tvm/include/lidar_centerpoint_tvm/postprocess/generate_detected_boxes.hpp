// Copyright 2022 AutoCore Ltd., TIER IV, Inc.
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

#ifndef LIDAR_CENTERPOINT_TVM__POSTPROCESS__GENERATE_DETECTED_BOXES_HPP_
#define LIDAR_CENTERPOINT_TVM__POSTPROCESS__GENERATE_DETECTED_BOXES_HPP_

#include <lidar_centerpoint_tvm/centerpoint_config.hpp>
#include <lidar_centerpoint_tvm/utils.hpp>

#include <vector>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{
void generateDetectedBoxes3D(
  const std::vector<float> & out_heatmap, const std::vector<float> & out_offset,
  const std::vector<float> & out_z, const std::vector<float> & out_dim,
  const std::vector<float> & out_rot, const std::vector<float> & out_vel,
  const CenterPointConfig & config, std::vector<Box3D> & det_boxes3d);

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_CENTERPOINT_TVM__POSTPROCESS__GENERATE_DETECTED_BOXES_HPP_
