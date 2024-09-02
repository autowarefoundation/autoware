// Copyright 2019 Christopher Ho
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
#include "time_utils/time_utils.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace time_utils
{
////////////////////////////////////////////////////////////////////////////////
std::chrono::nanoseconds interpolate(
  std::chrono::nanoseconds a, std::chrono::nanoseconds b, float t) noexcept
{
  // TODO(c.ho) consider long double
  const auto t_ = static_cast<double>(std::min(std::max(t, 0.0F), 1.0F));
  const auto del = std::chrono::duration_cast<std::chrono::duration<double>>(b - a);
  const auto del_ = std::chrono::duration_cast<std::chrono::nanoseconds>(t_ * del);
  return a + del_;
}

}  // namespace time_utils
