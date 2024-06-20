// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__DISTANCE__DISTANCE_HPP_
#define AUTOWARE__MOTION_UTILS__DISTANCE__DISTANCE_HPP_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <tuple>
#include <vector>

namespace autoware::motion_utils
{
std::optional<double> calcDecelDistWithJerkAndAccConstraints(
  const double current_vel, const double target_vel, const double current_acc, const double acc_min,
  const double jerk_acc, const double jerk_dec);

}  // namespace autoware::motion_utils

#endif  // AUTOWARE__MOTION_UTILS__DISTANCE__DISTANCE_HPP_
