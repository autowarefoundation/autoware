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

#include "interpolation/spherical_linear_interpolation.hpp"

namespace interpolation
{
geometry_msgs::msg::Quaternion slerp(
  const geometry_msgs::msg::Quaternion & src_quat, const geometry_msgs::msg::Quaternion & dst_quat,
  const double ratio)
{
  tf2::Quaternion src_tf;
  tf2::Quaternion dst_tf;
  tf2::fromMsg(src_quat, src_tf);
  tf2::fromMsg(dst_quat, dst_tf);
  const auto interpolated_quat = tf2::slerp(src_tf, dst_tf, ratio);
  return tf2::toMsg(interpolated_quat);
}

std::vector<geometry_msgs::msg::Quaternion> slerp(
  const std::vector<double> & base_keys,
  const std::vector<geometry_msgs::msg::Quaternion> & base_values,
  const std::vector<double> & query_keys)
{
  // throw exception for invalid arguments
  const auto validated_query_keys = interpolation_utils::validateKeys(base_keys, query_keys);
  interpolation_utils::validateKeysAndValues(base_keys, base_values);

  // calculate linear interpolation
  std::vector<geometry_msgs::msg::Quaternion> query_values;
  size_t key_index = 0;
  for (const auto query_key : validated_query_keys) {
    while (base_keys.at(key_index + 1) < query_key) {
      ++key_index;
    }

    const auto src_quat = base_values.at(key_index);
    const auto dst_quat = base_values.at(key_index + 1);
    const double ratio = (query_key - base_keys.at(key_index)) /
                         (base_keys.at(key_index + 1) - base_keys.at(key_index));

    const auto interpolated_quat = slerp(src_quat, dst_quat, ratio);
    query_values.push_back(interpolated_quat);
  }

  return query_values;
}

}  // namespace interpolation
