// Copyright 2018-2019 Autoware Foundation
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

#ifndef PURE_PURSUIT__UTIL__INTERPOLATE_HPP_
#define PURE_PURSUIT__UTIL__INTERPOLATE_HPP_

#include <cmath>
#include <iostream>
#include <vector>

namespace pure_pursuit
{
class LinearInterpolate
{
public:
  LinearInterpolate() {}
  ~LinearInterpolate() {}
  static bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value);
};

class SplineInterpolate
{
  bool initialized_ = false;
  std::vector<double> a_;  //!< @brief temporal vector for calculation
  std::vector<double> b_;  //!< @brief temporal vector for calculation
  std::vector<double> c_;  //!< @brief temporal vector for calculation
  std::vector<double> d_;  //!< @brief temporal vector for calculation

public:
  SplineInterpolate();
  explicit SplineInterpolate(const std::vector<double> & x);
  ~SplineInterpolate();
  void generateSpline(const std::vector<double> & x);
  double getValue(const double & s);
  bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value);
  void getValueVector(const std::vector<double> & s_v, std::vector<double> & value_v);
};
}  // namespace pure_pursuit

#endif  // PURE_PURSUIT__UTIL__INTERPOLATE_HPP_
