// Copyright 2021 Tier IV, Inc.
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

#include "tier4_autoware_utils/math/range.hpp"

#include <gtest/gtest.h>

#include <vector>

template <class T>
void expect_near_vector(
  const std::vector<T> & input, const std::vector<T> & expect, const T abs_error = 1e-6)
{
  ASSERT_EQ(input.size(), expect.size()) << "unequal length";

  for (size_t i = 0; i < input.size(); i++) {
    const auto x = input.at(i);
    const auto y = expect.at(i);
    EXPECT_NEAR(x, y, abs_error) << "differ at index " << i << ":" << x << ", " << y;
  }
}

void expect_eq_vector(const std::vector<int> & input, const std::vector<int> & expect)
{
  ASSERT_EQ(input.size(), expect.size()) << "unequal length";

  for (size_t i = 0; i < input.size(); ++i) {
    const auto x = input.at(i);
    const auto y = expect.at(i);
    EXPECT_EQ(x, y) << "differ at index " << i << ": " << x << ", " << y;
  }
}

TEST(arange_Test, arange_double)
{
  using tier4_autoware_utils::arange;

  // general cases
  {
    expect_near_vector(arange(0.0, 5.0, 1.0), std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0});
    expect_near_vector(arange(0.0, 5.1, 1.0), std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0});
    expect_near_vector(arange(2.0, 5.0, 0.5), std::vector<double>{2.0, 2.5, 3.0, 3.5, 4.0, 4.5});
    expect_near_vector(
      arange(0.1, 2.0, 0.2), std::vector<double>{0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9});

    // argument step is omitted.
    expect_near_vector(arange(0.0, 5.0), std::vector<double>{0.0, 1.0, 2.0, 3.0, 4.0});
  }
  // corner cases
  {
    // stop == start
    expect_near_vector(arange(3.0, 3.0, 1.0), std::vector<double>{});
    // step > stop - start
    expect_near_vector(arange(2.0, 4.0, 10.0), std::vector<double>{2.0});
    // step is negative value and stop < start
    expect_near_vector(
      arange(3.0, -3.0, -1.0), std::vector<double>{3.0, 2.0, 1.0, 0.0, -1.0, -2.0});
  }
  // invalid cases
  {
    // step == 0
    EXPECT_THROW(arange(0.0, 5.0, 0.0), std::invalid_argument);
    EXPECT_THROW(arange(0.0, -5.0, 0.0), std::invalid_argument);

    // positive step but start > stop.
    EXPECT_THROW(arange(0.0, -1.0, 0.1), std::invalid_argument);
    // negative step but start < stop.
    EXPECT_THROW(arange(0.0, 1.0, -0.1), std::invalid_argument);
  }
}

TEST(arange_Test, arange_float)
{
  using tier4_autoware_utils::arange;

  // general cases
  {
    expect_near_vector(arange(0.0f, 5.0f, 1.0f), std::vector<float>{0.0, 1.0, 2.0, 3.0, 4.0});
    expect_near_vector(arange(0.0f, 5.1f, 1.0f), std::vector<float>{0.0, 1.0, 2.0, 3.0, 4.0, 5.0});
    expect_near_vector(arange(2.0f, 5.0f, 0.5f), std::vector<float>{2.0, 2.5, 3.0, 3.5, 4.0, 4.5});
    expect_near_vector(
      arange(0.1f, 2.0f, 0.2f),
      std::vector<float>{0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9});

    // argument step is omitted.
    expect_near_vector(arange(0.0f, 5.0f), std::vector<float>{0.0, 1.0, 2.0, 3.0, 4.0});
  }
  // corner cases
  {
    // stop == start
    expect_near_vector(arange(3.0f, 3.0f, 1.0f), std::vector<float>{});
    // step > stop - start
    expect_near_vector(arange(2.0f, 4.0f, 10.0f), std::vector<float>{2.0});
    // step is negative value and stop < start
    expect_near_vector(
      arange(3.0f, -3.0f, -1.0f), std::vector<float>{3.0, 2.0, 1.0, 0.0, -1.0, -2.0});
  }
  // invalid cases
  {
    // step == 0
    EXPECT_THROW(arange(0.0f, 5.0f, 0.0f), std::invalid_argument);
    EXPECT_THROW(arange(0.0f, -5.0f, 0.0f), std::invalid_argument);

    // positive step but start > stop.
    EXPECT_THROW(arange(0.0f, -1.0f, 0.1f), std::invalid_argument);
    // negative step but start < stop.
    EXPECT_THROW(arange(0.0f, 1.0f, -0.1f), std::invalid_argument);
  }
}

TEST(arange_Test, arange_int)
{
  using tier4_autoware_utils::arange;

  // general cases
  {
    expect_eq_vector(arange(0, 5, 2), std::vector<int>{0, 2, 4});

    // argument step is omitted.
    expect_eq_vector(arange(0, 5), std::vector<int>{0, 1, 2, 3, 4});
  }
  // corner cases
  {
    // stop == start
    expect_eq_vector(arange(3, 3, 1), std::vector<int>{});
    // step > stop - start
    expect_eq_vector(arange(2, 4, 10), std::vector<int>{2});
    // step is negative value and stop < start
    expect_eq_vector(arange(3, -3, -1), std::vector<int>{3, 2, 1, 0, -1, -2});
  }
  // invalid cases
  {
    // step == 0
    EXPECT_THROW(arange(0, 5, 0), std::invalid_argument);
    EXPECT_THROW(arange(0, -5, 0), std::invalid_argument);

    // positive step but start > stop.
    EXPECT_THROW(arange(0, -3, 1), std::invalid_argument);
    // negative step but start < stop.
    EXPECT_THROW(arange(0, 3, -1), std::invalid_argument);
  }
}

TEST(test_linspace, linspace_double)
{
  using tier4_autoware_utils::linspace;

  // general cases
  {
    expect_near_vector(linspace(3.0, 7.0, 5), std::vector<double>{3.0, 4.0, 5.0, 6.0, 7.0});
    expect_near_vector(linspace(1.0, 2.0, 3), std::vector<double>{1.0, 1.5, 2.0});
    expect_near_vector(linspace(-1.0, 3.0, 5), std::vector<double>{-1.0, 0.0, 1.0, 2.0, 3.0});
    expect_near_vector(
      linspace(0.1, 1.0, 10),
      std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0});
  }
  // corner cases
  {
    // num == 2
    expect_near_vector(linspace(0.0, 10.0, 2), std::vector<double>{0.0, 10.0});
    // num == 1
    expect_near_vector(linspace(11.0, 20.0, 1), std::vector<double>{11.0});
    // num == 0
    expect_near_vector(linspace(30.0, 40.0, 0), std::vector<double>{});
    // start == stop
    expect_near_vector(linspace(2.3, 2.3, 3), std::vector<double>{2.3, 2.3, 2.3});
    // start > stop
    expect_near_vector(linspace(10.0, 5.0, 6), std::vector<double>{10.0, 9.0, 8.0, 7.0, 6.0, 5.0});
  }
}

TEST(test_linspace, linspace_float)
{
  using tier4_autoware_utils::linspace;

  // general cases
  {
    expect_near_vector(linspace(3.0f, 7.0f, 5), std::vector<double>{3.0, 4.0, 5.0, 6.0, 7.0});
    expect_near_vector(linspace(1.0f, 2.0f, 3), std::vector<double>{1.0, 1.5, 2.0});
    expect_near_vector(linspace(-1.0f, 3.0f, 5), std::vector<double>{-1.0, 0.0, 1.0, 2.0, 3.0});
    expect_near_vector(
      linspace(0.1f, 1.0f, 10),
      std::vector<double>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0});
  }
  // corner cases
  {
    // num == 2
    expect_near_vector(linspace(0.0f, 10.0f, 2), std::vector<double>{0.0, 10.0});
    // num == 1
    expect_near_vector(linspace(11.0f, 20.0f, 1), std::vector<double>{11.0});
    // num == 0
    expect_near_vector(linspace(30.0f, 40.0f, 0), std::vector<double>{});
    // start == stop
    expect_near_vector(linspace(2.3f, 2.3f, 3), std::vector<double>{2.3, 2.3, 2.3});
    // start > stop
    expect_near_vector(
      linspace(10.0f, 5.0f, 6), std::vector<double>{10.0, 9.0, 8.0, 7.0, 6.0, 5.0});
  }
}

TEST(test_linspace, linspace_int)
{
  using tier4_autoware_utils::linspace;

  // general cases
  {
    expect_near_vector(linspace(3, 7, 5), std::vector<double>{3.0, 4.0, 5.0, 6.0, 7.0});
    expect_near_vector(linspace(1, 2, 3), std::vector<double>{1.0, 1.5, 2.0});
    expect_near_vector(linspace(-1, 3, 5), std::vector<double>{-1.0, 0.0, 1.0, 2.0, 3.0});
  }
  // corner cases
  {
    // num == 2
    expect_near_vector(linspace(0, 10, 2), std::vector<double>{0.0, 10.0});
    // num == 1
    expect_near_vector(linspace(11, 20, 1), std::vector<double>{11.0});
    // num == 0
    expect_near_vector(linspace(30, 40, 0), std::vector<double>{});
    // start == stop
    expect_near_vector(linspace(3, 3, 3), std::vector<double>{3.0, 3.0, 3.0});
    // start > stop
    expect_near_vector(linspace(10, 5, 6), std::vector<double>{10.0, 9.0, 8.0, 7.0, 6.0, 5.0});
  }
}
