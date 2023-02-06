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
inline std::vector<size_t> calc_closest_segment_indices(
  const std::vector<double> & base_keys, const std::vector<double> & query_keys,
  const double overlap_threshold = 1e-3)
{
  // throw exception for invalid arguments
  const auto validated_query_keys = interpolation_utils::validateKeys(base_keys, query_keys);

  std::vector<size_t> closest_segment_indices(validated_query_keys.size());
  size_t closest_segment_idx = 0;
  for (size_t i = 0; i < validated_query_keys.size(); ++i) {
    // Check if query_key is closes to the terminal point of the base keys
    if (base_keys.back() - overlap_threshold < validated_query_keys.at(i)) {
      closest_segment_idx = base_keys.size() - 1;
    } else {
      for (size_t j = base_keys.size() - 1; j > closest_segment_idx; --j) {
        if (
          base_keys.at(j - 1) - overlap_threshold < validated_query_keys.at(i) &&
          validated_query_keys.at(i) < base_keys.at(j)) {
          // find closest segment in base keys
          closest_segment_idx = j - 1;
          break;
        }
      }
    }

    closest_segment_indices.at(i) = closest_segment_idx;
  }

  return closest_segment_indices;
}

template <class T>
std::vector<T> zero_order_hold(
  const std::vector<double> & base_keys, const std::vector<T> & base_values,
  const std::vector<size_t> & closest_segment_indices)
{
  // throw exception for invalid arguments
  interpolation_utils::validateKeysAndValues(base_keys, base_values);

  std::vector<T> query_values(closest_segment_indices.size());
  for (size_t i = 0; i < closest_segment_indices.size(); ++i) {
    query_values.at(i) = base_values.at(closest_segment_indices.at(i));
  }

  return query_values;
}

template <class T>
std::vector<T> zero_order_hold(
  const std::vector<double> & base_keys, const std::vector<T> & base_values,
  const std::vector<double> & query_keys, const double overlap_threshold = 1e-3)
{
  return zero_order_hold(
    base_keys, base_values, calc_closest_segment_indices(base_keys, query_keys, overlap_threshold));
}
}  // namespace interpolation

#endif  // INTERPOLATION__ZERO_ORDER_HOLD_HPP_
