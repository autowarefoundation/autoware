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

#include "interpolation/linear_interpolation.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

constexpr double epsilon = 1e-6;

TEST(linear_interpolation, lerp_scalar)
{
  EXPECT_EQ(interpolation::lerp(0.0, 1.0, 0.3), 0.3);
  EXPECT_EQ(interpolation::lerp(-0.5, 12.3, 0.3), 3.34);
}

TEST(linear_interpolation, lerp_vector)
{
  {  // straight: query_keys is same as base_keys
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0, 4.5, 6.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values = interpolation::lerp(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // straight: query_keys is random
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    const std::vector<double> base_values{0.0, 1.5, 3.0, 4.5, 6.0};
    const std::vector<double> query_keys{0.0, 0.7, 1.9, 4.0};
    const std::vector<double> ans{0.0, 1.05, 2.85, 6.0};

    const auto query_values = interpolation::lerp(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is same as base_keys
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys = base_keys;
    const std::vector<double> ans = base_values;

    const auto query_values = interpolation::lerp(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }

  {  // curve: query_keys is same as random
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys{0.0, 8.0, 18.0};
    const std::vector<double> ans{-0.18, 1.12, 1.4};

    const auto query_values = interpolation::lerp(base_keys, base_values, query_keys);
    for (size_t i = 0; i < query_values.size(); ++i) {
      EXPECT_NEAR(query_values.at(i), ans.at(i), epsilon);
    }
  }
}

TEST(linear_interpolation, lerp_scalar_query)
{
  {  // curve: query_keys is same as random
    const std::vector<double> base_keys{-1.5, 1.0, 5.0, 10.0, 15.0, 20.0};
    const std::vector<double> base_values{-1.2, 0.5, 1.0, 1.2, 2.0, 1.0};
    const std::vector<double> query_keys{0.0, 8.0, 18.0};
    const std::vector<double> ans{-0.18, 1.12, 1.4};

    for (size_t i = 0; i < query_keys.size(); ++i) {
      const auto query_value = interpolation::lerp(base_keys, base_values, query_keys.at(i));
      EXPECT_NEAR(query_value, ans.at(i), epsilon);
    }
  }
}
