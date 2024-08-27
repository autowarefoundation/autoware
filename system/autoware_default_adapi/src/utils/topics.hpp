// Copyright 2024 The Autoware Contributors
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

#ifndef UTILS__TOPICS_HPP_
#define UTILS__TOPICS_HPP_

#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <optional>

namespace autoware::default_adapi::utils
{

template <class MsgT>
MsgT ignore_stamp(MsgT msg)
{
  msg.stamp = rclcpp::Time(0, 0);
  return msg;
};

template <class PubT, class MsgT, class FuncT>
void notify(PubT & pub, std::optional<MsgT> & prev, const MsgT & curr, FuncT && ignore)
{
  if (!prev || ignore(*prev) != ignore(curr)) {
    prev = curr;
    pub->publish(curr);
  }
}

}  // namespace autoware::default_adapi::utils

#endif  // UTILS__TOPICS_HPP_
