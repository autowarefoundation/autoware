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

#include <utilization/interpolate.hpp>
#include <utilization/util.hpp>

#include <vector>

namespace behavior_velocity_planner
{
namespace interpolation
{
// TODO(murooka) delete these functions
/*
 * linear interpolation
 */
bool LinearInterpolate::interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index, std::vector<double> & return_value)
{
  if (!isValidInput(base_index, base_value, return_index)) {
    std::cerr << "[interpolate] invalid input. interpolation failed." << std::endl;
    return false;
  }

  // calculate linear interpolation
  int i = 0;
  for (const auto idx : return_index) {
    if (base_index[i] == idx) {
      return_value.push_back(base_value[i]);
      continue;
    }
    while (base_index[i] < idx) {
      ++i;
    }
    if (i <= 0 || static_cast<int>(base_index.size()) - 1 < i) {
      std::cerr << "[interpolate] undesired condition. skip this idx!" << std::endl;
      continue;
    }

    const double base_dist = base_index[i] - base_index[i - 1];
    const double to_forward = base_index[i] - idx;
    const double to_backward = idx - base_index[i - 1];
    if (to_forward < 0.0 || to_backward < 0.0) {
      std::cerr << "[interpolate] undesired condition. skip this idx!!" << std::endl;
      std::cerr << "i = " << i << ", base_index[i - 1] = " << base_index[i - 1] << ", idx = " << idx
                << ", base_index[i] = " << base_index[i] << std::endl;
      continue;
    }

    const double value = (to_backward * base_value[i] + to_forward * base_value[i - 1]) / base_dist;
    return_value.push_back(value);
  }
  return true;
}

/*
 * helper functions
 */
bool isIncrease(const std::vector<double> & x)
{
  for (int i = 0; i < static_cast<int>(x.size()) - 1; ++i) {
    if (x[i] > x[i + 1]) {
      return false;
    }
  }
  return true;
}

bool isValidInput(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index)
{
  if (base_index.empty() || base_value.empty() || return_index.empty()) {
    std::cout << "bad index : some vector is empty. base_index: " << base_index.size()
              << ", base_value: " << base_value.size() << ", return_index: " << return_index.size()
              << std::endl;
    return false;
  }
  if (!isIncrease(base_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. base_index = ["
              << base_index.front() << ", " << base_index.back() << "]" << std::endl;
    return false;
  }
  if (!isIncrease(return_index)) {
    std::cout << "bad index : base_index is not monotonically increasing. return_index = ["
              << return_index.front() << ", " << return_index.back() << "]" << std::endl;
    return false;
  }
  if (return_index.front() < base_index.front()) {
    std::cout << "bad index : return_index.front() < base_index.front()" << std::endl;
    return false;
  }
  if (base_index.back() < return_index.back()) {
    std::cout << "bad index : base_index.back() < return_index.back()" << std::endl;
    return false;
  }
  if (base_index.size() != base_value.size()) {
    std::cout << "bad index : base_index.size() != base_value.size()" << std::endl;
    return false;
  }

  return true;
}

std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y)
{
  if (x.size() != y.size()) {
    std::cerr << "x y vector size should be the same." << std::endl;
  }

  std::vector<double> dist_v;
  dist_v.push_back(0.0);
  for (unsigned int i = 0; i < x.size() - 1; ++i) {
    const double dx = x.at(i + 1) - x.at(i);
    const double dy = y.at(i + 1) - y.at(i);
    dist_v.push_back(dist_v.at(i) + std::hypot(dx, dy));
  }

  return dist_v;
}
}  // namespace interpolation
}  // namespace behavior_velocity_planner
