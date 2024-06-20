// Copyright 2023 Tier IV, Inc.
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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__PARAMETER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__PARAMETER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::universe_utils
{
template <class T>
T getOrDeclareParameter(rclcpp::Node & node, const std::string & name)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }

  return node.declare_parameter<T>(name);
}
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__PARAMETER_HPP_
