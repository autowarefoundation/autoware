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

#include "autoware_auto_common/helper_functions/template_utils.hpp"

#include <gtest/gtest.h>

struct CorrectType
{
};
struct FalseType
{
};

struct Foo
{
  static CorrectType bar(CorrectType, const CorrectType &, CorrectType *) { return CorrectType{}; }
};

template <template <typename> class Expression, typename... Ts>
using expression_valid_with_return =
  ::autoware::common::helper_functions::expression_valid_with_return<Expression, Ts...>;
template <template <typename> class Expression, typename... Ts>
using expression_valid = ::autoware::common::helper_functions::expression_valid<Expression, Ts...>;

// Types are defined here and not in the header because these definitions are basically the test
// code themselves.

// Correct way to call Foo::bar(...)
template <typename FooT, typename In1, typename In2, typename In3>
using call_bar_expression = decltype(std::declval<FooT>().bar(
  std::declval<In1>(), std::declval<const In2 &>(), std::declval<In3 *>()));

// Another correct way to call Foo::bar(...) since a temporary can bind to the const lvalue
// reference
template <typename FooT, typename In1, typename In2, typename In3>
using call_bar_expression2 = decltype(std::declval<FooT>().bar(
  std::declval<In1>(), std::declval<In2>(), std::declval<In3 *>()));

// Signature mismatch:
template <typename FooT, typename In1, typename In2, typename In3>
using false_bar_expression1 =
  decltype(std::declval<FooT>().bar(std::declval<In1>(), std::declval<In2>(), std::declval<In3>()));

// Signature mismatch:
template <typename FooT, typename In1, typename In2>
using false_bar_expression2 =
  decltype(std::declval<FooT>().bar(std::declval<In1>(), std::declval<const In2 &>()));

// cspell: ignore asdasd
// Signature mismatch:
template <typename FooT, typename In1, typename In2, typename In3>
using false_bar_expression3 = decltype(std::declval<FooT>().asdasd(
  std::declval<In1>(), std::declval<const In2 &>(), std::declval<In3 *>()));

// Correct signature, correct types:
template <typename FooT>
using correct_expression1 = call_bar_expression<FooT, CorrectType, CorrectType, CorrectType>;
template <typename FooT>
using correct_expression2 = call_bar_expression2<FooT, CorrectType, CorrectType, CorrectType>;

// Correct signature, false types:
template <typename FooT>
using false_expression1 = call_bar_expression<FooT, FalseType, CorrectType, CorrectType>;
template <typename FooT>
using false_expression2 = call_bar_expression<FooT, CorrectType, FalseType, CorrectType>;
template <typename FooT>
using false_expression3 = call_bar_expression<FooT, FalseType, FalseType, FalseType>;

// False signature, correct types:
template <typename FooT>
using false_expression4 = false_bar_expression1<FooT, CorrectType, CorrectType, CorrectType>;
template <typename FooT>
using false_expression5 = false_bar_expression3<FooT, CorrectType, CorrectType, CorrectType>;

// False signature, false types:
template <typename FooT>
using false_expression6 = false_bar_expression1<FooT, CorrectType, CorrectType, CorrectType>;
template <typename FooT>
using false_expression7 = false_bar_expression2<FooT, CorrectType, CorrectType>;

TEST(TestTemplateUtils, ExpressionValid)
{
  EXPECT_TRUE((expression_valid<correct_expression1, Foo>::value));
  EXPECT_TRUE((expression_valid<correct_expression2, Foo>::value));
  EXPECT_FALSE((expression_valid<false_expression1, Foo>::value));
  EXPECT_FALSE((expression_valid<false_expression2, Foo>::value));
  EXPECT_FALSE((expression_valid<false_expression3, Foo>::value));
  EXPECT_FALSE((expression_valid<false_expression4, Foo>::value));
  EXPECT_FALSE((expression_valid<false_expression5, Foo>::value));
  EXPECT_FALSE((expression_valid<false_expression6, Foo>::value));
  EXPECT_FALSE((expression_valid<false_expression7, Foo>::value));
}

TEST(TestTemplateUtils, ExpressionReturnValid)
{
  EXPECT_TRUE((expression_valid_with_return<correct_expression1, Foo, CorrectType>::value));
  EXPECT_FALSE((expression_valid_with_return<correct_expression1, Foo, FalseType>::value));

  EXPECT_TRUE((expression_valid_with_return<correct_expression2, Foo, CorrectType>::value));
  EXPECT_FALSE((expression_valid_with_return<correct_expression2, Foo, FalseType>::value));

  // If an expression is not valid, returning the right type will not be enough.
  EXPECT_FALSE((expression_valid_with_return<false_expression1, Foo, CorrectType>::value));
  EXPECT_FALSE((expression_valid_with_return<false_expression2, Foo, CorrectType>::value));
  EXPECT_FALSE((expression_valid_with_return<false_expression3, Foo, CorrectType>::value));
  EXPECT_FALSE((expression_valid_with_return<false_expression4, Foo, CorrectType>::value));
  EXPECT_FALSE((expression_valid_with_return<false_expression5, Foo, CorrectType>::value));
  EXPECT_FALSE((expression_valid_with_return<false_expression6, Foo, CorrectType>::value));
  EXPECT_FALSE((expression_valid_with_return<false_expression7, Foo, CorrectType>::value));
}
