//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "raw_vehicle_cmd_converter/interpolate.hpp"

#include <vector>

/*
 * linear interpolation
 */

namespace raw_vehicle_cmd_converter
{
bool LinearInterpolate::interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const double & return_index, double & return_value)
{
  auto isIncrease = [](const std::vector<double> & x) {
    for (size_t i = 0; i < x.size() - 1; ++i) {
      if (x[i] > x[i + 1]) {
        return false;
      }
    }
    return true;
  };

  if (base_index.size() == 0 || base_value.size() == 0) {
    printf(
      "[interpolate] some vector size is zero: base_index.size() = %lu, base_value.size() = %lu",
      base_index.size(), base_value.size());
    return false;
  }

  // check if inputs are valid
  if (
    !isIncrease(base_index) || return_index < base_index.front() ||
    base_index.back() < return_index || base_index.size() != base_value.size()) {
    std::cerr << "[isIncrease] bad index, return false" << std::endl;
    bool b1 = !isIncrease(base_index);
    bool b3 = return_index < base_index.front();
    bool b4 = base_index.back() < return_index;
    bool b5 = base_index.size() != base_value.size();
    printf("%d, %d, %d, %d\n", b1, b3, b4, b5);
    printf("base_index = [%f, %f]\n", base_index.front(), base_index.back());
    printf(
      "base_index.size() = %lu, base_value.size() = %lu\n", base_index.size(), base_value.size());
    printf("base_index: [");
    for (size_t i = 0; i < base_index.size(); ++i) {
      printf("%f, ", base_index.at(i));
    }
    printf("]\n");
    printf("base_value: [");
    for (size_t i = 0; i < base_value.size(); ++i) {
      printf("%f, ", base_value.at(i));
    }
    printf("]\n");
    printf("return_index = %f\n", return_index);
    return false;
  }

  // calculate linear interpolation
  size_t i = 0;
  if (base_index[i] == return_index) {
    return_value = base_value[i];
    return true;
  }
  while (base_index[i] < return_index) {
    ++i;
  }
  if (i <= 0 || base_index.size() - 1 < i) {
    std::cerr << "? something wrong. skip this return_index." << std::endl;
    return false;
  }

  const double dist_base_return_index = base_index[i] - base_index[i - 1];
  const double dist_to_forward = base_index[i] - return_index;
  const double dist_to_backward = return_index - base_index[i - 1];
  if (dist_to_forward < 0.0 || dist_to_backward < 0.0) {
    std::cerr << "?? something wrong. skip this return_index. base_index[i - 1] = "
              << base_index[i - 1] << ", return_index = " << return_index
              << ", base_index[i] = " << base_index[i] << std::endl;
    return false;
  }

  const double value = (dist_to_backward * base_value[i] + dist_to_forward * base_value[i - 1]) /
                       dist_base_return_index;
  return_value = value;

  return true;
}
}  // namespace raw_vehicle_cmd_converter
