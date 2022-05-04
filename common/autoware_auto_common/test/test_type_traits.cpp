// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Developed by Apex.AI, Inc.

#include <common/type_traits.hpp>
#include <common/types.hpp>

#include <gtest/gtest.h>

#include <tuple>

namespace
{
/// @brief      A simple testing function to check if all types are arithmetic.
///
///             Trait "is_arithmetic" is picked at random and any other trait could have been used
///             instead.
template <typename... Ts>
bool all_are_arithmetic()
{
  // This is just a random function that we use with conjunction.
  return autoware::common::type_traits::conjunction<std::is_arithmetic<Ts>...>::value;
}

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

}  // namespace

/// @test       Test that index of a type can be computed.
TEST(TestCommonTypeTraits, Index)
{
  using T = std::tuple<std::int32_t, float64_t>;
  EXPECT_EQ(0, (autoware::common::type_traits::index<std::int32_t, T>::value));
  EXPECT_EQ(1, (autoware::common::type_traits::index<float64_t, T>::value));
}

TEST(TestCommonTypeTraits, Conjunction)
{
  EXPECT_TRUE((all_are_arithmetic<std::int32_t, float32_t>()));
  EXPECT_FALSE(
    (all_are_arithmetic<std::int32_t, float32_t, std::tuple<std::int32_t, float32_t>>()));
}

TEST(TestCommonTypeTraits, Visit)
{
  const std::tuple<std::int32_t, float64_t> t;
  std::int32_t counter{};
  autoware::common::type_traits::visit(t, [&counter](const auto &) { counter++; });
  EXPECT_EQ(2, counter);
  float64_t sum{};
  autoware::common::type_traits::visit(
    std::make_tuple(2, 42.0F, 23.0),
    [&sum](const auto & element) { sum += static_cast<float64_t>(element); });
  EXPECT_DOUBLE_EQ(67.0, sum);
}

TEST(TestCommonTypeTraits, HasType)
{
  struct T1
  {
  };
  struct T2
  {
  };
  struct T3
  {
  };
  EXPECT_TRUE((autoware::common::type_traits::has_type<T1, std::tuple<T1, T2>>::value));
  EXPECT_FALSE((autoware::common::type_traits::has_type<T3, std::tuple<T1, T2>>::value));
  EXPECT_FALSE((autoware::common::type_traits::has_type<T1, std::tuple<>>::value));
}

TEST(TestCommonTypeTraits, TypeIntersection)
{
  struct T1
  {
  };
  struct T2
  {
  };
  struct T3
  {
  };
  using A = std::tuple<T1, T2>;
  using B = std::tuple<T2, T3>;
  using C = std::tuple<T3>;
  EXPECT_TRUE(
    (std::is_same<std::tuple<T2>, autoware::common::type_traits::intersect<A, B>::type>::value));
  EXPECT_TRUE((std::is_same<A, autoware::common::type_traits::intersect<A, A>::type>::value));
  EXPECT_TRUE(
    (std::is_same<std::tuple<>, autoware::common::type_traits::intersect<A, C>::type>::value));
}
