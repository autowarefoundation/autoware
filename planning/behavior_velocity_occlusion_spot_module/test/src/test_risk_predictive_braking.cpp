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

#include "occlusion_spot_utils.hpp"
#include "risk_predictive_braking.hpp"
#include "utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

TEST(safeMotion, delay_jerk_acceleration)
{
  namespace utils = behavior_velocity_planner::occlusion_spot_utils;
  using utils::calculateSafeMotion;
  /**
   * @brief check if calculation is correct in below parameter
   * delay =  0.5 [s]
   * a_max = -4.5 [m/s^2]
   * j_max = -3.0 [m/s^3]
   * case1 delay
   * case2 delay + jerk
   * case3 delay + jerk + acc
   */
  utils::Velocity v{};
  v.safety_ratio = 1.0;
  v.max_stop_jerk = -3.0;
  v.max_stop_accel = -4.5;
  v.delay_time = 0.5;
  double ttc = 0.0;
  const double eps = 1e-3;
  // case 1 delay
  {
    ttc = 0.5;
    utils::SafeMotion sm = utils::calculateSafeMotion(v, ttc);
    EXPECT_NEAR(sm.safe_velocity, 0.0, eps);
    EXPECT_NEAR(sm.stop_dist, 0.0, eps);
  }
  // case 2 delay + jerk
  {
    ttc = 1.5;
    utils::SafeMotion sm = utils::calculateSafeMotion(v, ttc);
    EXPECT_NEAR(sm.safe_velocity, 1.5, eps);
    EXPECT_NEAR(sm.stop_dist, 1.25, eps);
  }
  // case 3 delay + jerk + acc
  {
    ttc = 3.25;
    utils::SafeMotion sm = utils::calculateSafeMotion(v, ttc);
    EXPECT_NEAR(sm.safe_velocity, 9, eps);
    EXPECT_NEAR(std::round(sm.stop_dist * 100.0) / 100.0, 13.92, eps);
  }
}
