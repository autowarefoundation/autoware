// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__SYSTEM__STOP_WATCH_HPP_
#define AUTOWARE__UNIVERSE_UTILS__SYSTEM__STOP_WATCH_HPP_

#include <chrono>
#include <string>
#include <unordered_map>

namespace autoware::universe_utils
{
template <
  class OutputUnit = std::chrono::seconds, class InternalUnit = std::chrono::microseconds,
  class Clock = std::chrono::steady_clock>
class StopWatch
{
public:
  StopWatch() { tic(default_name); }

  void tic(const std::string & name = default_name) { t_start_[name] = Clock::now(); }

  void tic(const char * name) { tic(std::string(name)); }

  double toc(const std::string & name, const bool reset = false)
  {
    const auto t_start = t_start_.at(name);
    const auto t_end = Clock::now();
    const auto duration = std::chrono::duration_cast<InternalUnit>(t_end - t_start).count();

    if (reset) {
      t_start_[name] = Clock::now();
    }

    const auto one_sec = std::chrono::duration_cast<InternalUnit>(OutputUnit(1)).count();

    return static_cast<double>(duration) / one_sec;
  }

  double toc(const char * name, const bool reset = false) { return toc(std::string(name), reset); }

  double toc(const bool reset = false) { return toc(default_name, reset); }

private:
  using Time = std::chrono::time_point<Clock>;
  static constexpr const char * default_name{"__auto__"};

  std::unordered_map<std::string, Time> t_start_;
};
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__SYSTEM__STOP_WATCH_HPP_
