// Copyright 2021 - 2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "control_performance_analysis/control_performance_analysis_utils.hpp"

#include <algorithm>

namespace control_performance_analysis
{
namespace utils
{
double determinant(std::array<double, 2> const & a, std::array<double, 2> const & b)
{
  return a[0] * b[1] - b[0] * a[1];
}

}  // namespace utils
}  // namespace control_performance_analysis
