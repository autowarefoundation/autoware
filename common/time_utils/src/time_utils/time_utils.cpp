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
namespace details
{
template <typename TimeT>
TimeT duration_to_msg(std::chrono::nanoseconds dt)
{
  // Round down seconds
  const auto negative = decltype(dt)::zero() > dt;
  if (negative) {
    dt -= std::chrono::seconds(1);
  }
  // rounds towards zero
  const auto dt_sec = std::chrono::duration_cast<std::chrono::seconds>(dt);
  TimeT ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  // Clamp value to receptive field of retval seconds
  {
    using SecT = decltype(ret.sec);
    using Rep = decltype(dt_sec)::rep;
    const auto sec_max = static_cast<Rep>(std::numeric_limits<SecT>::max());
    const auto sec_min = static_cast<Rep>(std::numeric_limits<SecT>::min());
    const auto secs = std::min(std::max(sec_min, dt_sec.count()), sec_max);
    ret.sec = static_cast<SecT>(secs);
  }
  // Get raw nanoseconds
  const auto dt_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(dt);
  auto nanosec = dt_nsec - std::chrono::duration_cast<std::chrono::nanoseconds>(dt_sec);
  if (negative) {
    nanosec += std::chrono::seconds(1);
    if (0 > nanosec.count()) {
      throw std::logic_error{"Bug: Nanosec should never be negative"};
    }
  }
  // Clamp down to receptive field of retval nanoseconds
  {
    using NsecT = decltype(ret.nanosec);
    using Rep = decltype(nanosec)::rep;
    const auto nsec_max = static_cast<Rep>(std::numeric_limits<NsecT>::max());
    const auto nsec_min = static_cast<Rep>(std::numeric_limits<NsecT>::min());
    const auto nanosec_clamp = std::min(std::max(nsec_min, nanosec.count()), nsec_max);
    ret.nanosec = static_cast<NsecT>(nanosec_clamp);
  }
  return ret;
}
}  // namespace details

////////////////////////////////////////////////////////////////////////////////
builtin_interfaces::msg::Time to_message(const std::chrono::system_clock::time_point t)
{
  const auto dt = t.time_since_epoch();
  return details::duration_to_msg<builtin_interfaces::msg::Time>(dt);
}

////////////////////////////////////////////////////////////////////////////////
builtin_interfaces::msg::Duration to_message(std::chrono::nanoseconds dt)
{
  return details::duration_to_msg<builtin_interfaces::msg::Duration>(dt);
}

////////////////////////////////////////////////////////////////////////////////
std::chrono::system_clock::time_point from_message(builtin_interfaces::msg::Time t) noexcept
{
  const auto dt_ns = std::chrono::seconds(t.sec) + std::chrono::nanoseconds(t.nanosec);
  const auto dt = std::chrono::duration_cast<std::chrono::nanoseconds>(dt_ns);
  return std::chrono::system_clock::time_point{} + dt;
}

////////////////////////////////////////////////////////////////////////////////
std::chrono::nanoseconds from_message(builtin_interfaces::msg::Duration dt) noexcept
{
  const auto ns = std::chrono::nanoseconds{dt.nanosec};
  const auto us = std::chrono::duration_cast<std::chrono::nanoseconds>(ns);
  return std::chrono::seconds{dt.sec} + us;
}

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
