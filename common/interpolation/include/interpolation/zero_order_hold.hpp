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
template <class T>
std::vector<T> zero_order_hold(
  const std::vector<double> & base_keys, const std::vector<T> & base_values,
  const std::vector<double> & query_keys, const double overlap_threshold = 1e-3)
{
  // throw exception for invalid arguments
  const auto validated_query_keys = interpolation_utils::validateKeys(base_keys, query_keys);
  interpolation_utils::validateKeysAndValues(base_keys, base_values);

  std::vector<T> query_values;
  size_t closest_segment_idx = 0;
  for (size_t i = 0; i < validated_query_keys.size(); ++i) {
    // Check if query_key is closes to the terminal point of the base keys
    if (base_keys.back() - overlap_threshold < validated_query_keys.at(i)) {
      closest_segment_idx = base_keys.size() - 1;
    } else {
      for (size_t j = closest_segment_idx; j < base_keys.size() - 1; ++j) {
        if (
          base_keys.at(j) - overlap_threshold < validated_query_keys.at(i) &&
          validated_query_keys.at(i) < base_keys.at(j + 1)) {
          // find closest segment in base keys
          closest_segment_idx = j;
        }
      }
    }

    query_values.push_back(base_values.at(closest_segment_idx));
  }

  return query_values;
}
}  // namespace interpolation

#endif  // INTERPOLATION__ZERO_ORDER_HOLD_HPP_
