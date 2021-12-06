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

#include "motion_velocity_smoother/linear_interpolation.hpp"

#include <vector>

namespace motion_velocity_smoother
{
/*
 * linear interpolation
 */
namespace linear_interpolation
{
boost::optional<std::vector<double>> interpolate(
  const std::vector<double> & sample_x, const std::vector<double> & sample_value,
  const std::vector<double> & query_x)
{
  std::vector<double> query_value;
  auto isIncrease = [](const std::vector<double> & x) {
    if (x.empty()) {
      return false;
    }
    for (size_t i = 0; i < x.size() - 1; ++i) {
      if (x.at(i) > x.at(i + 1)) {
        return false;
      }
    }
    return true;
  };

  if (sample_x.empty() || sample_value.empty() || query_x.empty()) {
    printf(
      "[interpolate] some vector size is zero: sample_x.size() = %lu, sample_value.size() = %lu, "
      "query_x.size() = %lu\n",
      sample_x.size(), sample_value.size(), query_x.size());
    return {};
  }

  // check if inputs are valid
  if (
    !isIncrease(sample_x) || !isIncrease(query_x) || query_x.front() < sample_x.front() ||
    sample_x.back() < query_x.back() || sample_x.size() != sample_value.size()) {
    std::cerr << "[isIncrease] bad index, return false" << std::endl;
    const bool b1 = !isIncrease(sample_x);
    const bool b2 = !isIncrease(query_x);
    const bool b3 = query_x.front() < sample_x.front();
    const bool b4 = sample_x.back() < query_x.back();
    const bool b5 = sample_x.size() != sample_value.size();
    printf("%d, %d, %d, %d, %d\n", b1, b2, b3, b4, b5);
    printf("sample_x = [%f, %f]\n", sample_x.front(), sample_x.back());
    printf("query_x = [%f, %f]\n", query_x.front(), query_x.back());
    printf(
      "sample_x.size() = %lu, sample_value.size() = %lu\n", sample_x.size(), sample_value.size());
    return {};
  }

  // calculate linear interpolation
  int i = 0;
  for (const auto idx : query_x) {
    if (sample_x.at(i) == idx) {
      query_value.push_back(sample_value.at(i));
      continue;
    }
    while (sample_x.at(i) < idx) {
      ++i;
    }
    if (i <= 0 || static_cast<int>(sample_x.size()) - 1 < i) {
      std::cerr << "? something wrong. skip this idx." << std::endl;
      continue;
    }

    const double dist_base_idx = sample_x.at(i) - sample_x.at(i - 1);
    const double dist_to_forward = sample_x.at(i) - idx;
    const double dist_to_backward = idx - sample_x.at(i - 1);
    if (dist_to_forward < 0.0 || dist_to_backward < 0.0) {
      std::cerr << "?? something wrong. skip this idx. sample_x.at(i - 1) = " << sample_x.at(i - 1)
                << ", idx = " << idx << ", sample_x.at(i) = " << sample_x.at(i) << std::endl;
      continue;
    }

    const double value =
      (dist_to_backward * sample_value.at(i) + dist_to_forward * sample_value.at(i - 1)) /
      dist_base_idx;
    query_value.push_back(value);
  }
  return query_value;
}
}  // namespace linear_interpolation
}  // namespace motion_velocity_smoother
