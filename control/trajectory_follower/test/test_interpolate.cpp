// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>

#include "common/types.hpp"
#include "gtest/gtest.h"
#include "trajectory_follower/interpolate.hpp"

using autoware::common::types::float64_t;
TEST(TestInterpolate, Nominal) {
  using autoware::motion::control::trajectory_follower::linearInterpolate;

  // Simple case
  {
    std::vector<float64_t> original_indexes = {1.0, 2.0, 3.0};
    std::vector<float64_t> original_values = {1.0, 2.0, 3.0};
    std::vector<float64_t> target_indexes = {1.5, 2.5};
    std::vector<float64_t> target_values;

    ASSERT_TRUE(
      linearInterpolate(
        original_indexes, original_values, target_indexes,
        target_values));
    ASSERT_EQ(target_values.size(), target_indexes.size());
    for (size_t i = 0; i < target_values.size(); ++i) {
      EXPECT_EQ(target_values[i], target_indexes[i]);
    }
  }
  // Non regular indexes
  {
    std::vector<float64_t> original_indexes = {1.0, 1.5, 3.0};
    std::vector<float64_t> original_values = {1.0, 2.0, 3.5};
    std::vector<float64_t> target_indexes = {1.25, 2.5, 3.0};
    std::vector<float64_t> target_values;

    ASSERT_TRUE(
      linearInterpolate(
        original_indexes, original_values, target_indexes,
        target_values));
    ASSERT_EQ(target_values.size(), target_indexes.size());
    EXPECT_EQ(target_values[0], 1.5);
    EXPECT_EQ(target_values[1], 3.0);
    EXPECT_EQ(target_values[2], 3.5);
  }
  // Single index query
  {
    std::vector<float64_t> original_indexes = {1.0, 1.5, 3.0};
    std::vector<float64_t> original_values = {1.0, 2.0, 3.5};
    float64_t target_index = 1.25;
    float64_t target_value;

    ASSERT_TRUE(
      linearInterpolate(
        original_indexes, original_values, target_index,
        target_value));
    EXPECT_EQ(target_value, 1.5);
  }
}
TEST(TestInterpolate, Failure) {
  using autoware::motion::control::trajectory_follower::linearInterpolate;

  std::vector<float64_t> target_values;

  // Non increasing indexes
  ASSERT_FALSE(
    linearInterpolate(
      {1.0, 2.0, 1.5, 3.0}, {1.0, 2.0, 3.0, 4.0},
      {1.0, 3.0}, target_values));
  ASSERT_FALSE(
    linearInterpolate(
      {1.0, 2.0, 2.5, 3.0}, {1.0, 2.0, 3.0, 4.0},
      {3.0, 1.0}, target_values));

  // Target indexes out of range
  ASSERT_FALSE(
    linearInterpolate(
      {1.0, 2.0, 2.5, 3.0}, {1.0, 2.0, 3.0, 4.0},
      {0.0, 3.0}, target_values));
  ASSERT_FALSE(
    linearInterpolate(
      {1.0, 2.0, 2.5, 3.0}, {1.0, 2.0, 3.0, 4.0},
      {1.0, 3.5}, target_values));

  // Missmatched inputs
  ASSERT_FALSE(
    linearInterpolate(
      {1.0, 2.0, 2.5, 3.0}, {1.0, 2.0, 3.0},
      {1.0, 1.5}, target_values));
  ASSERT_FALSE(
    linearInterpolate(
      {1.0, 2.0, 3.0}, {1.0, 2.0, 3.0, 4.0},
      {1.0, 1.5}, target_values));

  // Input size 0
  ASSERT_FALSE(linearInterpolate({}, {}, {1.0, 3.5}, target_values));

  // Input size 1
  ASSERT_FALSE(linearInterpolate({1.5}, {1.5}, {1.0, 3.5}, target_values));
}
