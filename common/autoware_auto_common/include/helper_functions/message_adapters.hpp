// Copyright 2017-2019 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief This file includes common helper functions

#ifndef HELPER_FUNCTIONS__MESSAGE_ADAPTERS_HPP_
#define HELPER_FUNCTIONS__MESSAGE_ADAPTERS_HPP_

#include <builtin_interfaces/msg/time.hpp>

#include <string>

namespace autoware
{
namespace common
{
namespace helper_functions
{
namespace message_field_adapters
{
/// Using alias for Time message
using TimeStamp = builtin_interfaces::msg::Time;

/// \brief Helper class to check existance of header file in compile time:
/// https://stackoverflow.com/a/16000226/2325407
template <typename T, typename = nullptr_t>
struct HasHeader : std::false_type
{
};

template <typename T>
struct HasHeader<T, decltype((void)T::header, nullptr)> : std::true_type
{
};

/////////// Template declarations

/// Get frame id from message. nullptr_t is used to prevent template ambiguity on
/// SFINAE specializations. Provide a default value on specializations for a friendly API.
/// \tparam T Message type.
/// \param msg Message.
/// \return Frame id of the message.
template <typename T, nullptr_t>
const std::string & get_frame_id(const T & msg) noexcept;

/// Get a reference to the frame id from message. nullptr_t is used to prevent
/// template ambiguity on SFINAE specializations. Provide a default value on
/// specializations for a friendly API.
/// \tparam T Message type.
/// \param msg Message.
/// \return Frame id of the message.
template <typename T, nullptr_t>
std::string & get_frame_id(T & msg) noexcept;

/// Get stamp from message. nullptr_t is used to prevent template ambiguity on
/// SFINAE specializations. Provide a default value on specializations for a friendly API.
/// \tparam T Message type.
/// \param msg Message.
/// \return Frame id of the message.
template <typename T, nullptr_t>
const TimeStamp & get_stamp(const T & msg) noexcept;

/// Get a reference to the stamp from message. nullptr_t is used to prevent
/// template ambiguity on SFINAE specializations. Provide a default value on
/// specializations for a friendly API.
/// \tparam T Message type.
/// \param msg Message.
/// \return Frame id of the message.
template <typename T, nullptr_t>
TimeStamp & get_stamp(T & msg) noexcept;

/////////////// Default specializations for message types that contain a header.
template <class T, typename std::enable_if<HasHeader<T>::value, nullptr_t>::type = nullptr>
const std::string & get_frame_id(const T & msg) noexcept
{
  return msg.header.frame_id;
}

template <class T, typename std::enable_if<HasHeader<T>::value, nullptr_t>::type = nullptr>
std::string & get_frame_id(T & msg) noexcept
{
  return msg.header.frame_id;
}

template <class T, typename std::enable_if<HasHeader<T>::value, nullptr_t>::type = nullptr>
TimeStamp & get_stamp(T & msg) noexcept
{
  return msg.header.stamp;
}

template <class T, typename std::enable_if<HasHeader<T>::value, nullptr_t>::type = nullptr>
TimeStamp get_stamp(const T & msg) noexcept
{
  return msg.header.stamp;
}

}  // namespace message_field_adapters
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__MESSAGE_ADAPTERS_HPP_
