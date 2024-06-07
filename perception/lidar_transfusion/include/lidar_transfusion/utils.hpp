// Copyright 2024 TIER IV, Inc.
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

#ifndef LIDAR_TRANSFUSION__UTILS_HPP_
#define LIDAR_TRANSFUSION__UTILS_HPP_

#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

namespace lidar_transfusion
{

struct Box3D
{
  int label;
  float score;
  float x;
  float y;
  float z;
  float width;
  float length;
  float height;
  float yaw;
};

struct CloudInfo
{
  uint32_t x_offset;
  uint32_t y_offset;
  uint32_t z_offset;
  uint32_t intensity_offset;
  uint8_t x_datatype;
  uint8_t y_datatype;
  uint8_t z_datatype;
  uint8_t intensity_datatype;
  uint8_t x_count;
  uint8_t y_count;
  uint8_t z_count;
  uint8_t intensity_count;
  uint32_t point_step;
  bool is_bigendian;

  CloudInfo()
  : x_offset(0),
    y_offset(4),
    z_offset(8),
    intensity_offset(12),
    x_datatype(7),
    y_datatype(7),
    z_datatype(7),
    intensity_datatype(7),
    x_count(1),
    y_count(1),
    z_count(1),
    intensity_count(1),
    point_step(16),
    is_bigendian(false)
  {
  }

  bool operator!=(const CloudInfo & rhs) const
  {
    return x_offset != rhs.x_offset || y_offset != rhs.y_offset || z_offset != rhs.z_offset ||
           intensity_offset != rhs.intensity_offset || x_datatype != rhs.x_datatype ||
           y_datatype != rhs.y_datatype || z_datatype != rhs.z_datatype ||
           intensity_datatype != rhs.intensity_datatype || x_count != rhs.x_count ||
           y_count != rhs.y_count || z_count != rhs.z_count ||
           intensity_count != rhs.intensity_count || is_bigendian != rhs.is_bigendian;
  }
};

enum NetworkIO { voxels = 0, num_points, coors, cls_score, dir_pred, bbox_pred, ENUM_SIZE };

// cspell: ignore divup
template <typename T1, typename T2>
unsigned int divup(const T1 a, const T2 b)
{
  if (a == 0) {
    throw std::runtime_error("A dividend of divup isn't positive.");
  }
  if (b == 0) {
    throw std::runtime_error("A divisor of divup isn't positive.");
  }

  return (a + b - 1) / b;
}

}  // namespace lidar_transfusion

#endif  // LIDAR_TRANSFUSION__UTILS_HPP_
