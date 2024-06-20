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

#ifndef AUTOWARE__UNIVERSE_UTILS__MATH__SIN_TABLE_HPP_
#define AUTOWARE__UNIVERSE_UTILS__MATH__SIN_TABLE_HPP_

#include <cstddef>

namespace autoware::universe_utils
{

constexpr size_t sin_table_size = 32769;
constexpr size_t discrete_arcs_num_90 = 32768;
constexpr size_t discrete_arcs_num_360 = 131072;
extern const float g_sin_table[sin_table_size];

}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__MATH__SIN_TABLE_HPP_
