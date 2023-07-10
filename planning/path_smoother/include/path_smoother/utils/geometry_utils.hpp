// Copyright 2023 TIER IV, Inc.
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

#ifndef PATH_SMOOTHER__UTILS__GEOMETRY_UTILS_HPP_
#define PATH_SMOOTHER__UTILS__GEOMETRY_UTILS_HPP_

#include <tier4_autoware_utils/geometry/geometry.hpp>

namespace path_smoother
{
namespace geometry_utils
{
template <typename T1, typename T2>
bool isSamePoint(const T1 & t1, const T2 & t2)
{
  const auto p1 = tier4_autoware_utils::getPoint(t1);
  const auto p2 = tier4_autoware_utils::getPoint(t2);

  constexpr double epsilon = 1e-6;
  return (std::abs(p1.x - p2.x) <= epsilon && std::abs(p1.y - p2.y) <= epsilon);
}
}  // namespace geometry_utils
}  // namespace path_smoother
#endif  // PATH_SMOOTHER__UTILS__GEOMETRY_UTILS_HPP_
