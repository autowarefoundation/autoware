// Copyright 2023 TIER IV, Inc.
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

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"

#include <gtest/gtest.h>

// Test the CostTranslationTable and InverseCostTranslationTable functions
using autoware::occupancy_grid_map::cost_value::cost_translation_table;
using autoware::occupancy_grid_map::cost_value::inverse_cost_translation_table;

TEST(CostTranslationTableTest, TestRange)
{
  // Test the value range of CostTranslationTable
  for (int i = 0; i < 256; i++) {
    EXPECT_GE(cost_translation_table[i], 1);
    EXPECT_LE(cost_translation_table[i], 99);
  }

  // Test the value range of InverseCostTranslationTable
  for (int i = 1; i < 100; i++) {
    EXPECT_GE(inverse_cost_translation_table[i], 0);
    EXPECT_LE(inverse_cost_translation_table[i], 255);
  }
}

TEST(CostTranslationTableTest, TestConversion)
{
  // Test the forward and inverse conversion of 0, 128, and 255
  EXPECT_EQ(cost_translation_table[0], 1);
  EXPECT_EQ(cost_translation_table[128], 50);
  EXPECT_EQ(cost_translation_table[255], 99);
  EXPECT_EQ(inverse_cost_translation_table[1], 2);
  EXPECT_EQ(inverse_cost_translation_table[50], 128);
  EXPECT_EQ(inverse_cost_translation_table[99], 255);
}
