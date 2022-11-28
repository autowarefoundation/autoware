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

#include "lidar_centerpoint_tvm/utils.hpp"

#include <stdexcept>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{

std::size_t divup(const std::size_t a, const std::size_t b)
{
  if (a == 0) {
    throw std::runtime_error("A dividend of divup isn't positive.");
  }
  if (b == 0) {
    throw std::runtime_error("A divisor of divup isn't positive.");
  }

  return (a + b - 1) / b;
}

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware
