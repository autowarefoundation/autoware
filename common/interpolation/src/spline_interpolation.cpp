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

#include "interpolation/spline_interpolation.hpp"

#include "interpolation/interpolation_utils.hpp"

#include <vector>

namespace
{
// solve Ax = d
// where A is tridiagonal matrix
//     [b_0 c_0 ...                       ]
//     [a_0 b_1 c_1 ...               O   ]
// A = [            ...                   ]
//     [   O         ... a_N-3 b_N-2 c_N-2]
//     [                   ... a_N-2 b_N-1]
struct TDMACoef
{
  explicit TDMACoef(const size_t num_row)
  {
    a.resize(num_row - 1);
    b.resize(num_row);
    c.resize(num_row - 1);
    d.resize(num_row);
  }

  std::vector<double> a;
  std::vector<double> b;
  std::vector<double> c;
  std::vector<double> d;
};

std::vector<double> solveTridiagonalMatrixAlgorithm(const TDMACoef & tdma_coef)
{
  const auto & a = tdma_coef.a;
  const auto & b = tdma_coef.b;
  const auto & c = tdma_coef.c;
  const auto & d = tdma_coef.d;

  const size_t num_row = b.size();

  std::vector<double> x(num_row);
  if (num_row != 1) {
    // calculate p and q
    std::vector<double> p;
    std::vector<double> q;
    p.push_back(-c[0] / b[0]);
    q.push_back(d[0] / b[0]);

    for (size_t i = 1; i < num_row; ++i) {
      const double den = b[i] + a[i - 1] * p[i - 1];
      p.push_back(-c[i - 1] / den);
      q.push_back((d[i] - a[i - 1] * q[i - 1]) / den);
    }

    // calculate solution
    x[num_row - 1] = q[num_row - 1];

    for (size_t i = 1; i < num_row; ++i) {
      const size_t j = num_row - 1 - i;
      x[j] = p[j] * x[j + 1] + q[j];
    }
  } else {
    x.push_back(d[0] / b[0]);
  }

  return x;
}

interpolation::MultiSplineCoef generateSplineCoefficients(
  const std::vector<double> & base_keys, const std::vector<double> & base_values)
{
  const size_t num_base = base_keys.size();  // N+1

  std::vector<double> diff_keys;    // N
  std::vector<double> diff_values;  // N
  for (size_t i = 0; i < num_base - 1; ++i) {
    diff_keys.push_back(base_keys.at(i + 1) - base_keys.at(i));
    diff_values.push_back(base_values.at(i + 1) - base_values.at(i));
  }

  std::vector<double> v = {0.0};
  if (num_base > 2) {
    // solve tridiagonal matrix algorithm
    TDMACoef tdma_coef(num_base - 2);  // N-1

    for (size_t i = 0; i < num_base - 2; ++i) {
      tdma_coef.b[i] = 2 * (diff_keys[i] + diff_keys[i + 1]);
      if (i != num_base - 3) {
        tdma_coef.a[i] = diff_keys[i + 1];
        tdma_coef.c[i] = diff_keys[i + 1];
      }
      tdma_coef.d[i] =
        6.0 * (diff_values[i + 1] / diff_keys[i + 1] - diff_values[i] / diff_keys[i]);
    }

    const std::vector<double> tdma_res = solveTridiagonalMatrixAlgorithm(tdma_coef);

    // calculate v
    v.insert(v.end(), tdma_res.begin(), tdma_res.end());
  }
  v.push_back(0.0);

  // calculate a, b, c, d of spline coefficients
  interpolation::MultiSplineCoef multi_spline_coef(num_base - 1);  // N
  for (size_t i = 0; i < num_base - 1; ++i) {
    multi_spline_coef.a[i] = (v[i + 1] - v[i]) / 6.0 / diff_keys[i];
    multi_spline_coef.b[i] = v[i] / 2.0;
    multi_spline_coef.c[i] =
      diff_values[i] / diff_keys[i] - diff_keys[i] * (2 * v[i] + v[i + 1]) / 6.0;
    multi_spline_coef.d[i] = base_values[i];
  }

  return multi_spline_coef;
}

std::vector<double> getSplineInterpolatedValues(
  const std::vector<double> & base_keys, const std::vector<double> & query_keys,
  const interpolation::MultiSplineCoef & multi_spline_coef)
{
  const auto & a = multi_spline_coef.a;
  const auto & b = multi_spline_coef.b;
  const auto & c = multi_spline_coef.c;
  const auto & d = multi_spline_coef.d;

  std::vector<double> res;
  size_t j = 0;
  for (const auto & query_key : query_keys) {
    while (base_keys.at(j + 1) < query_key) {
      ++j;
    }

    const double ds = query_key - base_keys.at(j);
    res.push_back(d.at(j) + (c.at(j) + (b.at(j) + a.at(j) * ds) * ds) * ds);
  }

  return res;
}
}  // namespace

namespace interpolation
{
std::vector<double> slerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys)
{
  // throw exceptions for invalid arguments
  interpolation_utils::validateInput(base_keys, base_values, query_keys);

  // calculate spline coefficients
  const auto multi_spline_coef = generateSplineCoefficients(base_keys, base_values);

  // interpolate base_keys at query_keys
  return getSplineInterpolatedValues(base_keys, query_keys, multi_spline_coef);
}
}  // namespace interpolation
