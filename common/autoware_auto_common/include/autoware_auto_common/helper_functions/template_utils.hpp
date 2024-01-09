// Copyright 2021 the Autoware Foundation
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

#ifndef AUTOWARE_AUTO_COMMON__HELPER_FUNCTIONS__TEMPLATE_UTILS_HPP_
#define AUTOWARE_AUTO_COMMON__HELPER_FUNCTIONS__TEMPLATE_UTILS_HPP_

#include "autoware_auto_common/common/types.hpp"

#include <type_traits>

namespace autoware
{
namespace common
{
namespace helper_functions
{
/// This struct is `std::true_type` if the expression is valid for a given template and
/// `std::false_type` otherwise.
/// \tparam ExpressionTemplate Expression to be checked in compile time
/// \tparam T Template parameter to instantiate the expression.
template <template <typename...> class ExpressionTemplate, typename T, typename = void>
struct expression_valid : std::false_type
{
};

/// This struct is `std::true_type` if the expression is valid for a given template and
/// `std::false_type` otherwise.
/// \tparam ExpressionTemplate Expression to be checked in compile time
/// \tparam T Template parameter to instantiate the expression.
template <template <typename...> class ExpressionTemplate, typename T>
struct expression_valid<ExpressionTemplate, T, types::void_t<ExpressionTemplate<T>>>
: std::true_type
{
};

/// This struct is `std::true_type` if the expression is valid for a given template
/// type with the specified return type and `std::false_type` otherwise.
/// \tparam ExpressionTemplate Expression to be checked in compile time
/// \tparam T Template parameter to instantiate the expression.
/// \tparam ReturnT Return type of the expression.
template <
  template <typename...> class ExpressionTemplate, typename T, typename ReturnT, typename = void>
struct expression_valid_with_return : std::false_type
{
};

/// This struct is `std::true_type` if the expression is valid for a given template
/// type with the specified return type and `std::false_type` otherwise.
/// \tparam ExpressionTemplate Expression to be checked in compile time
/// \tparam T Template parameter to instantiate the expression.
/// \tparam ReturnT Return type of the expression.
template <template <typename...> class ExpressionTemplate, typename T, typename ReturnT>
struct expression_valid_with_return<
  ExpressionTemplate, T, ReturnT,
  std::enable_if_t<std::is_same<ReturnT, ExpressionTemplate<T>>::value>> : std::true_type
{
};

}  // namespace helper_functions
}  // namespace common
}  // namespace autoware
#endif  // AUTOWARE_AUTO_COMMON__HELPER_FUNCTIONS__TEMPLATE_UTILS_HPP_
