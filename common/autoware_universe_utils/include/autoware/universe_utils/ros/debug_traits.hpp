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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__DEBUG_TRAITS_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__DEBUG_TRAITS_HPP_

#include <tier4_debug_msgs/msg/bool_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>
#include <tier4_debug_msgs/msg/int64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/int64_stamped.hpp>
#include <tier4_debug_msgs/msg/string_stamped.hpp>

#include <type_traits>

namespace autoware::universe_utils::debug_traits
{
template <typename T>
struct is_debug_message : std::false_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::BoolStamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::Float32MultiArrayStamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::Float32Stamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::Float64MultiArrayStamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::Float64Stamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::Int32MultiArrayStamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::Int32Stamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::Int64MultiArrayStamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::Int64Stamped> : std::true_type
{
};

template <>
struct is_debug_message<tier4_debug_msgs::msg::StringStamped> : std::true_type
{
};
}  // namespace autoware::universe_utils::debug_traits

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__DEBUG_TRAITS_HPP_
