// Copyright 2022 The Autoware Contributors
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

#ifndef POSE_INITIALIZER__COPY_VECTOR_TO_ARRAY_HPP_
#define POSE_INITIALIZER__COPY_VECTOR_TO_ARRAY_HPP_

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <string>
#include <vector>

template <typename T, size_t N>
void copy_vector_to_array(const std::vector<T> & vector, std::array<T, N> & array)
{
  if (N != vector.size()) {
    // throws the error to prevent causing an anonymous bug
    // such as only partial array is initialized
    const auto v = std::to_string(vector.size());
    const auto n = std::to_string(N);
    throw std::invalid_argument(
      "Vector size (which is " + v + ") is different from the copy size (which is " + n + ")");
  }
  std::copy_n(vector.begin(), N, array.begin());
}

template <class NodeT>
std::array<double, 36> get_covariance_parameter(NodeT * node, const std::string & name)
{
  const auto parameter = node->template declare_parameter<std::vector<double>>(name);
  std::array<double, 36> covariance;
  copy_vector_to_array(parameter, covariance);
  return covariance;
}

#endif  // POSE_INITIALIZER__COPY_VECTOR_TO_ARRAY_HPP_
