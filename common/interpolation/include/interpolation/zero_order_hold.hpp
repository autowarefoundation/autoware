// Copyright 2022 Tier IV, Inc.
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

#ifndef INTERPOLATION__ZERO_ORDER_HOLD_HPP_
#define INTERPOLATION__ZERO_ORDER_HOLD_HPP_

#include "interpolation/interpolation_utils.hpp"

#include <vector>

namespace interpolation
{
std::vector<double> zero_order_hold(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys, const double overlap_threshold = 1e-3);
}  // namespace interpolation

#endif  // INTERPOLATION__ZERO_ORDER_HOLD_HPP_
