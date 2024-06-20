// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__MATH__NORMALIZATION_HPP_
#define AUTOWARE__UNIVERSE_UTILS__MATH__NORMALIZATION_HPP_

#include "autoware/universe_utils/math/constants.hpp"

#include <cmath>

namespace autoware::universe_utils
{
inline double normalizeDegree(const double deg, const double min_deg = -180)
{
  const auto max_deg = min_deg + 360.0;

  const auto value = std::fmod(deg, 360.0);
  if (min_deg <= value && value < max_deg) {
    return value;
  }

  return value - std::copysign(360.0, value);
}

inline double normalizeRadian(const double rad, const double min_rad = -pi)
{
  const auto max_rad = min_rad + 2 * pi;

  const auto value = std::fmod(rad, 2 * pi);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * pi, value);
}

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__MATH__NORMALIZATION_HPP_
