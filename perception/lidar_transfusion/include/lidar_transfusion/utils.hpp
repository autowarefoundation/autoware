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
