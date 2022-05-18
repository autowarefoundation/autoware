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
#include <vector>

template <typename T, size_t N>
void CopyVectorToArray(const std::vector<T> & vector, std::array<T, N> & array)
{
  if (N != vector.size()) {
    // throws the error to prevent causing an anonymous bug
    // such as only partial array is initialized
    throw std::invalid_argument(fmt::format(
      "Vector size (which is {}) is different from the copy size (which is {})", vector.size(), N));
  }

  std::copy_n(vector.begin(), N, array.begin());
}

#endif  // POSE_INITIALIZER__COPY_VECTOR_TO_ARRAY_HPP_
