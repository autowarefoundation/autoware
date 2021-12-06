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

#ifndef MOTION_VELOCITY_SMOOTHER__LINEAR_INTERPOLATION_HPP_
#define MOTION_VELOCITY_SMOOTHER__LINEAR_INTERPOLATION_HPP_

#include <boost/optional.hpp>

#include <cmath>
#include <iostream>
#include <vector>

namespace motion_velocity_smoother
{
namespace linear_interpolation
{
boost::optional<std::vector<double>> interpolate(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index);
}
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__LINEAR_INTERPOLATION_HPP_
