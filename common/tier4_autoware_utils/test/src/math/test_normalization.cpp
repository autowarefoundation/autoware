// Copyright 2020 Tier IV, Inc.
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

#include "tier4_autoware_utils/math/normalization.hpp"

#include <gtest/gtest.h>

TEST(normalization, normalizeDegree)
{
  using tier4_autoware_utils::normalizeDegree;

  // -180 <= deg < 180
  {
    constexpr double eps = 0.1;
    constexpr double v_min = -180;
    constexpr double v_mid = 0;
    constexpr double v_max = 180;

    EXPECT_DOUBLE_EQ(normalizeDegree(v_min - eps), v_max - eps);
    EXPECT_DOUBLE_EQ(normalizeDegree(v_min), v_min);
    EXPECT_DOUBLE_EQ(normalizeDegree(v_mid), v_mid);
    EXPECT_DOUBLE_EQ(normalizeDegree(v_max - eps), v_max - eps);
    EXPECT_DOUBLE_EQ(normalizeDegree(v_max), v_min);
  }

  // 0 <= deg < 360
  {
    constexpr double eps = 0.1;
    constexpr double v_min = 0;
    constexpr double v_mid = 180;
    constexpr double v_max = 360;

    EXPECT_DOUBLE_EQ(normalizeDegree(v_min - eps, 0), v_max - eps);
    EXPECT_DOUBLE_EQ(normalizeDegree(v_min, 0), v_min);
    EXPECT_DOUBLE_EQ(normalizeDegree(v_mid, 0), v_mid);
    EXPECT_DOUBLE_EQ(normalizeDegree(v_max - eps, 0), v_max - eps);
    EXPECT_DOUBLE_EQ(normalizeDegree(v_max, 0), v_min);
  }
}

TEST(normalization, normalizeRadian)
{
  using tier4_autoware_utils::normalizeRadian;

  // -M_PI <= deg < M_PI
  {
    constexpr double eps = 0.1;
    constexpr double v_min = -M_PI;
    constexpr double v_mid = 0;
    constexpr double v_max = M_PI;

    EXPECT_DOUBLE_EQ(normalizeRadian(v_min - eps), v_max - eps);
    EXPECT_DOUBLE_EQ(normalizeRadian(v_min), v_min);
    EXPECT_DOUBLE_EQ(normalizeRadian(v_mid), v_mid);
    EXPECT_DOUBLE_EQ(normalizeRadian(v_max - eps), v_max - eps);
    EXPECT_DOUBLE_EQ(normalizeRadian(v_max), v_min);
  }

  // 0 <= deg < 2 * M_PI
  {
    constexpr double eps = 0.1;
    constexpr double v_min = 0;
    constexpr double v_mid = M_PI;
    constexpr double v_max = 2 * M_PI;

    EXPECT_DOUBLE_EQ(normalizeRadian(v_min - eps, 0), v_max - eps);
    EXPECT_DOUBLE_EQ(normalizeRadian(v_min, 0), v_min);
    EXPECT_DOUBLE_EQ(normalizeRadian(v_mid, 0), v_mid);
    EXPECT_DOUBLE_EQ(normalizeRadian(v_max - eps, 0), v_max - eps);
    EXPECT_DOUBLE_EQ(normalizeRadian(v_max, 0), v_min);
  }
}
