// Copyright 2017-2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief This file includes common functionality for 3D geometry, such as dot products

#ifndef GEOMETRY__COMMON_3D_HPP_
#define GEOMETRY__COMMON_3D_HPP_

#include <geometry/common_2d.hpp>

namespace autoware
{
namespace common
{
namespace geometry
{

/// \tparam T1, T2 point type. Must have point adapters defined or have float members x, y and z
/// \brief compute p * q = p1 * q1 + p2 * q2 + p3 * 13
/// \param[in] pt first point
/// \param[in] q second point
/// \return 3d scalar dot product
template <typename T1, typename T2>
inline auto dot_3d(const T1 & pt, const T2 & q)
{
  using point_adapter::x_;
  using point_adapter::y_;
  using point_adapter::z_;
  return (x_(pt) * x_(q)) + (y_(pt) * y_(q) + z_(pt) * z_(q));
}

/// \brief Compute 3D squared euclidean distance between two points
/// \tparam OUT return type. Type of the returned distance.
/// \tparam T1 point type. Must have point adapters defined or have float members x y and z
/// \tparam T2 point type. Must have point adapters defined or have float members x y and z
/// \param a point 1
/// \param b point 2
/// \return squared 3D euclidean distance
template <typename OUT = float32_t, typename T1, typename T2>
inline OUT squared_distance_3d(const T1 & a, const T2 & b)
{
  const auto x = static_cast<OUT>(point_adapter::x_(a)) - static_cast<OUT>(point_adapter::x_(b));
  const auto y = static_cast<OUT>(point_adapter::y_(a)) - static_cast<OUT>(point_adapter::y_(b));
  const auto z = static_cast<OUT>(point_adapter::z_(a)) - static_cast<OUT>(point_adapter::z_(b));
  return (x * x) + (y * y) + (z * z);
}

/// \brief Compute 3D euclidean distance between two points
/// \tparam T1 point type. Must have point adapters defined or have float members x y and z
/// \tparam T2 point type. Must have point adapters defined or have float members x y and z
/// \param a point 1
/// \param b point 2
/// \return 3D euclidean distance
template <typename OUT = float32_t, typename T1, typename T2>
inline OUT distance_3d(const T1 & a, const T2 & b)
{
  return std::sqrt(squared_distance_3d<OUT>(a, b));
}

}  // namespace geometry
}  // namespace common
}  // namespace autoware

#endif  // GEOMETRY__COMMON_3D_HPP_
