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

#include <common/types.hpp>

#include <cmath>
#include <string>

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{
using autoware::common::types::float32_t;

/// \brief Project a point from a pointcloud to a 2D map.
/// \param[in] val Coordinate of the point in the pointcloud.
/// \param[in] ori Diameter of area containing the pointcloud.
/// \param[in] scale Scaling factor from pointcloud size to grid size.
/// \return The grid in which the point is.
inline int32_t F2I(float32_t val, float32_t ori, float32_t scale)
{
  return static_cast<int32_t>(std::floor((ori - val) * scale));
}

/// \brief Transform a pointcloud scale to a pixel scale.
/// \param[in] in_pc Coordinate of the point in the pointcloud.
/// \param[in] in_range Range of the pointcloud.
/// \param[in] out_size Size of the grid.
/// \return The distance to the point in pixel scale.
inline int32_t Pc2Pixel(float32_t in_pc, float32_t in_range, float32_t out_size)
{
  float32_t inv_res = 0.5f * out_size / in_range;
  return static_cast<int32_t>(std::floor((in_range - in_pc) * inv_res));
}

/// \brief Transform a pixel scale to a pointcloud scale.
/// \param[in] in_pixel Coordinate of the cell in the grid.
/// \param[in] in_size Size of the grid.
/// \param[in] out_range Range of the pointcloud.
/// \return The distance to the cell in pointcloud scale.
inline float32_t Pixel2Pc(int32_t in_pixel, float32_t in_size, float32_t out_range)
{
  float32_t res = 2.0f * out_range / in_size;
  return out_range - (static_cast<float32_t>(in_pixel) + 0.5f) * res;
}
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_APOLLO_SEGMENTATION_TVM__UTIL_HPP_
