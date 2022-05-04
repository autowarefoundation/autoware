// Copyright 2021 Apex.AI, Inc.
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

#ifndef HELPER_FUNCTIONS__TYPE_NAME_HPP_
#define HELPER_FUNCTIONS__TYPE_NAME_HPP_

#include <common/visibility_control.hpp>

#include <string>
#include <typeinfo>

#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)
#include <cxxabi.h>
#endif

namespace autoware
{
namespace helper_functions
{

/// @brief      Get a demangled name of a type.
template <typename T>
COMMON_PUBLIC std::string get_type_name()
{
#if defined(__clang__) || defined(__GNUC__) || defined(__GNUG__)
  return abi::__cxa_demangle(typeid(T).name(), NULL, NULL, 0);
#else
  // For unsupported compilers return a mangled name.
  return typeid(T).name();
#endif
}

/// @brief      Get a demangled name of a type given its instance.
template <typename T>
COMMON_PUBLIC std::string get_type_name(const T &)
{
  return get_type_name<T>();
}

}  // namespace helper_functions
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__TYPE_NAME_HPP_
