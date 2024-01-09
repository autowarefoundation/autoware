// Copyright 2020 Apex.AI, Inc.
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

#ifndef AUTOWARE_AUTO_COMMON__HELPER_FUNCTIONS__ANGLE_UTILS_HPP_
#define AUTOWARE_AUTO_COMMON__HELPER_FUNCTIONS__ANGLE_UTILS_HPP_

#include <cmath>
#include <type_traits>

namespace autoware
{
namespace common
{
namespace helper_functions
{

namespace detail
{
constexpr auto kDoublePi = 2.0 * M_PI;
}  // namespace detail

///
/// @brief      Wrap angle to the [-pi, pi] range.
///
/// @details    This method uses the formula suggested in the paper [On wrapping the Kalman filter
///             and estimating with the SO(2) group](https://arxiv.org/pdf/1708.05551.pdf) and
///             implements the following formula:
///             \f$\mathrm{mod}(\alpha + \pi, 2 \pi) - \pi\f$.
///
/// @param[in]  angle  The input angle
///
/// @tparam     T      Type of scalar
///
/// @return     Angle wrapped to the chosen range.
///
template <typename T>
constexpr T wrap_angle(T angle) noexcept
{
  auto help_angle = angle + T(M_PI);
  while (help_angle < T{}) {
    help_angle += T(detail::kDoublePi);
  }
  while (help_angle >= T(detail::kDoublePi)) {
    help_angle -= T(detail::kDoublePi);
  }
  return help_angle - T(M_PI);
}

}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // AUTOWARE_AUTO_COMMON__HELPER_FUNCTIONS__ANGLE_UTILS_HPP_
