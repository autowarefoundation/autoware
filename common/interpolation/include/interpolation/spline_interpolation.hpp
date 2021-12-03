// Copyright 2021 Tier IV, Inc.
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

#ifndef INTERPOLATION__SPLINE_INTERPOLATION_HPP_
#define INTERPOLATION__SPLINE_INTERPOLATION_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace interpolation
{
// NOTE: X(s) = a_i (s - s_i)^3 + b_i (s - s_i)^2 + c_i (s - s_i) + d_i : (i = 0, 1, ... N-1)
struct MultiSplineCoef
{
  explicit MultiSplineCoef(const size_t num_spline)
  {
    a.resize(num_spline);
    b.resize(num_spline);
    c.resize(num_spline);
    d.resize(num_spline);
  }

  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
};

std::vector<double> slerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys);
}  // namespace interpolation

#endif  // INTERPOLATION__SPLINE_INTERPOLATION_HPP_
