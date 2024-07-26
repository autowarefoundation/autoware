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

#include "autoware/probabilistic_occupancy_grid_map/fusion_policy/fusion_policy.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"

#include <gtest/gtest.h>

// Test the log-odds update rule
TEST(FusionPolicyTest, TestLogOddsUpdateRule)
{
  using autoware::occupancy_grid_map::fusion_policy::log_odds_fusion::logOddsFusion;
  const double MARGIN = 0.03;
  const double OCCUPIED = 1.0 - MARGIN;
  const double FREE = 0.0 + MARGIN;
  const double UNKNOWN = 0.5;
  const double EPSILON = 1e-3;

  // same case
  std::vector<double> case1_1 = {OCCUPIED, OCCUPIED};
  std::vector<double> case1_2 = {FREE, FREE};
  std::vector<double> case1_3 = {UNKNOWN, UNKNOWN};
  EXPECT_GE(logOddsFusion(case1_1), OCCUPIED);
  EXPECT_LE(logOddsFusion(case1_2), FREE);
  EXPECT_NEAR(logOddsFusion(case1_3), UNKNOWN, EPSILON);

  // with unknown
  std::vector<double> case2_1 = {OCCUPIED, UNKNOWN};
  std::vector<double> case2_2 = {FREE, UNKNOWN};
  EXPECT_NEAR(logOddsFusion(case2_1), OCCUPIED, EPSILON);
  EXPECT_NEAR(logOddsFusion(case2_2), FREE, EPSILON);

  // with conflict
  std::vector<double> case3_1 = {OCCUPIED, FREE};
  EXPECT_NEAR(logOddsFusion(case3_1), UNKNOWN, EPSILON);
}

// Test the dempster-shafer update rule
TEST(FusionPolicyTest, TestDempsterShaferUpdateRule)
{
  using autoware::occupancy_grid_map::fusion_policy::dempster_shafer_fusion::dempsterShaferFusion;
  const double MARGIN = 0.03;
  const double OCCUPIED = 1.0 - MARGIN;
  const double FREE = 0.0 + MARGIN;
  const double UNKNOWN = 0.5;
  const double EPSILON = 1e-3;

  // same case
  std::vector<double> case1_1 = {OCCUPIED, OCCUPIED};
  std::vector<double> case1_2 = {FREE, FREE};
  std::vector<double> case1_3 = {UNKNOWN, UNKNOWN};
  EXPECT_GE(dempsterShaferFusion(case1_1), OCCUPIED);
  EXPECT_LE(dempsterShaferFusion(case1_2), FREE);
  EXPECT_NEAR(dempsterShaferFusion(case1_3), UNKNOWN, EPSILON);

  // with unknown
  std::vector<double> case2_1 = {OCCUPIED, UNKNOWN};
  std::vector<double> case2_2 = {FREE, UNKNOWN};
  EXPECT_NEAR(dempsterShaferFusion(case2_1), OCCUPIED, EPSILON);
  EXPECT_NEAR(dempsterShaferFusion(case2_2), FREE, EPSILON);

  // with conflict
  std::vector<double> case3_1 = {OCCUPIED, FREE};
  EXPECT_NEAR(dempsterShaferFusion(case3_1), UNKNOWN, EPSILON);
}
