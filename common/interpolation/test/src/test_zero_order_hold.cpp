// Copyright 2022 Tier IV, Inc.
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

#include "interpolation/zero_order_hold.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

constexpr double epsilon = 1e-6;

TEST(zero_order_hold_interpolation, vector_interpolation)
{
  {  // straight: query_keys is same as base_keys
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 2.5, 3.5, 0.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: query_keys is random
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0, 4.5, 6.0};
    const std::vector<double> query_keys{0.0, 0.7, 1.9, 4.0};
    const std::vector<double> ans{0.0, 0.0, 1.5, 6.0};

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is same as base_keys
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is same as random
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys{0.0, 8.0, 18.0};
    const std::vector<double> ans{-1.2, 1.0, 2.0};

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // Boundary Condition
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 2.5, 3.5, 0.0};
    const std::vector<double> query_keys{0.0, 1.0, 2.0, 3.0, 4.0 - 0.001};
    const std::vector<double> ans = {0.0, 1.5, 2.5, 3.5, 3.5};

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // Boundary Condition
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 2.5, 3.5, 0.0};
    const std::vector<double> query_keys{0.0, 1.0, 2.0, 3.0, 4.0 - 0.0001};
    const std::vector<double> ans = {0.0, 1.5, 2.5, 3.5, 0.0};

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }
}

TEST(zero_order_hold_interpolation, vector_interpolation_no_double_interpolation)
{
  {  // straight: query_keys is same as base_keys
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<bool> base_values{true, true, false, true, true};
    const std::vector<double> query_keys = base_keys;
    const auto ans = base_values;

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_EQ(query_values.at(i), ans.at(i));
    }
  }

  {  // straight: query_keys is random
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{true, false, false, true, false};
    const std::vector<double> query_keys{0.0, 0.7, 1.9, 4.0};
    const std::vector<bool> ans = {true, true, false, false};

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_EQ(query_values.at(i), ans.at(i));
    }
  }

  {  // Boundary Condition
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<bool> base_values{true, true, false, true, false};
    const std::vector<double> query_keys{0.0, 1.0, 2.0, 3.0, 4.0 - 0.001};
    const std::vector<double> ans = {true, true, false, true, true};

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // Boundary Condition
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{true, false, true, true, false};
    const std::vector<double> query_keys{0.0, 1.0, 2.0, 3.0, 4.0 - 0.0001};
    const std::vector<double> ans = base_values;

    const auto query_values = interpolation::zero_order_hold(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }
}
