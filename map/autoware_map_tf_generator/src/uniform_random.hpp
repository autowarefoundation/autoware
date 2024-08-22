// Copyright 2022 TIER IV, Inc.
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

#ifndef UNIFORM_RANDOM_HPP_
#define UNIFORM_RANDOM_HPP_

#include <random>
#include <vector>

namespace autoware::map_tf_generator
{
std::vector<size_t> inline uniform_random(const size_t max_exclusive, const size_t n)
{
  std::default_random_engine generator;
  std::uniform_int_distribution<size_t> distribution(0, max_exclusive - 1);

  std::vector<size_t> v(n);
  for (size_t i = 0; i < n; i++) {
    v[i] = distribution(generator);
  }
  return v;
}
}  // namespace autoware::map_tf_generator

#endif  // UNIFORM_RANDOM_HPP_
