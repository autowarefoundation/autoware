// Copyright 2024 Tier IV, Inc.
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

#ifndef RESULT_HPP_
#define RESULT_HPP_

#include <utility>
#include <variant>

namespace autoware::behavior_velocity_planner
{

template <typename Ok, typename Error>
class Result
{
public:
  explicit Result(const Ok & ok) : data_(ok) {}
  explicit Result(const Error & err) : data_(err) {}
  explicit operator bool() const noexcept { return std::holds_alternative<Ok>(data_); }
  bool operator!() const noexcept { return !static_cast<bool>(*this); }
  const Ok & ok() const { return std::get<Ok>(data_); }
  const Error & err() const { return std::get<Error>(data_); }

private:
  std::variant<Ok, Error> data_;
};

template <typename Ok, typename Error, typename... Args>
Result<Ok, Error> make_ok(Args &&... args)
{
  return Result<Ok, Error>(Ok{std::forward<Args>(args)...});
}

template <typename Ok, typename Error, typename... Args>
Result<Ok, Error> make_err(Args &&... args)
{
  return Result<Ok, Error>(Error{std::forward<Args>(args)...});
}

}  // namespace autoware::behavior_velocity_planner

#endif  // RESULT_HPP_
