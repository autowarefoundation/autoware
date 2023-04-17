// Copyright 2017-2022 Arm Ltd., Autoware Foundation, The Apollo Authors
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

#ifndef LIDAR_APOLLO_SEGMENTATION_TVM__UTIL_HPP_
#define LIDAR_APOLLO_SEGMENTATION_TVM__UTIL_HPP_

#include <cmath>
#include <string>

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{

/// \brief Project a point from a pointcloud to a 2D map.
/// \param[in] val Coordinate of the point in the pointcloud.
/// \param[in] ori Diameter of area containing the pointcloud.
/// \param[in] scale Scaling factor from pointcloud size to grid size.
/// \return The grid in which the point is.
inline int32_t F2I(float val, float ori, float scale)
{
  return static_cast<int32_t>(std::floor((ori - val) * scale));
}

/// \brief Transform a pointcloud scale to a pixel scale.
/// \param[in] in_pc Coordinate of the point in the pointcloud.
/// \param[in] in_range Range of the pointcloud.
/// \param[in] out_size Size of the grid.
/// \return The distance to the point in pixel scale.
inline int32_t Pc2Pixel(float in_pc, float in_range, float out_size)
{
  float inv_res = 0.5f * out_size / in_range;
  return static_cast<int32_t>(std::floor((in_range - in_pc) * inv_res));
}

/// \brief Transform a pixel scale to a pointcloud scale.
/// \param[in] in_pixel Coordinate of the cell in the grid.
/// \param[in] in_size Size of the grid.
/// \param[in] out_range Range of the pointcloud.
/// \return The distance to the cell in pointcloud scale.
inline float Pixel2Pc(int32_t in_pixel, float in_size, float out_range)
{
  float res = 2.0f * out_range / in_size;
  return out_range - (static_cast<float>(in_pixel) + 0.5f) * res;
}
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_APOLLO_SEGMENTATION_TVM__UTIL_HPP_
