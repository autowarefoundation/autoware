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

#include "interpolation/interpolation_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

constexpr double epsilon = 1e-6;

TEST(interpolation_utils, isIncreasing)
{
  // empty
  const std::vector<double> empty_vec;
  EXPECT_THROW(interpolation_utils::isIncreasing(empty_vec), std::invalid_argument);

  // increase
  const std::vector<double> increasing_vec{0.0, 1.5, 3.0, 4.5, 6.0};
  EXPECT_EQ(interpolation_utils::isIncreasing(increasing_vec), true);

  // not decrease
  const std::vector<double> not_increasing_vec{0.0, 1.5, 1.5, 4.5, 6.0};
  EXPECT_EQ(interpolation_utils::isIncreasing(not_increasing_vec), false);

  // decrease
  const std::vector<double> decreasing_vec{0.0, 1.5, 1.2, 4.5, 6.0};
  EXPECT_EQ(interpolation_utils::isIncreasing(decreasing_vec), false);
}

TEST(interpolation_utils, isNotDecreasing)
{
  // empty
  const std::vector<double> empty_vec;
  EXPECT_THROW(interpolation_utils::isNotDecreasing(empty_vec), std::invalid_argument);

  // increase
  const std::vector<double> increasing_vec{0.0, 1.5, 3.0, 4.5, 6.0};
  EXPECT_EQ(interpolation_utils::isNotDecreasing(increasing_vec), true);

  // not decrease
  const std::vector<double> not_increasing_vec{0.0, 1.5, 1.5, 4.5, 6.0};
  EXPECT_EQ(interpolation_utils::isNotDecreasing(not_increasing_vec), true);

  // decrease
  const std::vector<double> decreasing_vec{0.0, 1.5, 1.2, 4.5, 6.0};
  EXPECT_EQ(interpolation_utils::isNotDecreasing(decreasing_vec), false);
}

TEST(interpolation_utils, validateInput)
{
  using interpolation_utils::validateInput;

  const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0};
  const std::vector<double> base_values{0.0, 1.0, 2.0, 3.0};
  const std::vector<double> query_keys{0.0, 1.0, 2.0, 3.0};

  // valid
  EXPECT_NO_THROW(validateInput(base_keys, base_values, query_keys));

  // empty
  const std::vector<double> empty_vec;
  EXPECT_THROW(validateInput(empty_vec, base_values, query_keys), std::invalid_argument);
  EXPECT_THROW(validateInput(base_keys, empty_vec, query_keys), std::invalid_argument);
  EXPECT_THROW(validateInput(base_keys, base_values, empty_vec), std::invalid_argument);

  // size is less than 2
  const std::vector<double> short_vec{0.0};
  EXPECT_THROW(validateInput(short_vec, base_values, query_keys), std::invalid_argument);
  EXPECT_THROW(validateInput(base_keys, short_vec, query_keys), std::invalid_argument);
  EXPECT_THROW(validateInput(short_vec, short_vec, query_keys), std::invalid_argument);

  // partly not increase
  const std::vector<double> partly_not_increasing_vec{0.0, 0.0, 2.0, 3.0};
  // NOTE: base_keys must be strictly monotonous increasing vector
  EXPECT_THROW(
    validateInput(partly_not_increasing_vec, base_values, query_keys), std::invalid_argument);
  // NOTE: query_keys is allowed to be monotonous non-decreasing vector
  EXPECT_NO_THROW(validateInput(base_keys, base_values, partly_not_increasing_vec));

  // decrease
  const std::vector<double> decreasing_vec{0.0, -1.0, 2.0, 3.0};
  EXPECT_THROW(validateInput(decreasing_vec, base_values, query_keys), std::invalid_argument);
  EXPECT_THROW(validateInput(base_keys, base_values, decreasing_vec), std::invalid_argument);

  // out of range
  const std::vector<double> front_out_query_keys{-1.0, 1.0, 2.0, 3.0};
  EXPECT_THROW(validateInput(base_keys, base_values, front_out_query_keys), std::invalid_argument);

  const std::vector<double> back_out_query_keys{0.0, 1.0, 2.0, 4.0};
  EXPECT_THROW(validateInput(base_keys, base_values, back_out_query_keys), std::invalid_argument);

  // size is different
  const std::vector<double> different_size_base_values{0.0, 1.0, 2.0};
  EXPECT_THROW(
    validateInput(base_keys, different_size_base_values, query_keys), std::invalid_argument);
}
