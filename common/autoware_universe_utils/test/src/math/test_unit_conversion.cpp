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

#include "autoware/universe_utils/math/unit_conversion.hpp"

#include <gtest/gtest.h>

using autoware::universe_utils::pi;

TEST(unit_conversion, deg2rad)
{
  using autoware::universe_utils::deg2rad;

  EXPECT_DOUBLE_EQ(deg2rad(-720), -4 * pi);
  EXPECT_DOUBLE_EQ(deg2rad(0), 0);
  EXPECT_DOUBLE_EQ(deg2rad(30), pi / 6);
  EXPECT_DOUBLE_EQ(deg2rad(60), pi / 3);
  EXPECT_DOUBLE_EQ(deg2rad(90), pi / 2);
  EXPECT_DOUBLE_EQ(deg2rad(180), pi);
  EXPECT_DOUBLE_EQ(deg2rad(360), 2 * pi);
}

TEST(unit_conversion, rad2deg)
{
  using autoware::universe_utils::rad2deg;

  EXPECT_DOUBLE_EQ(rad2deg(-4 * pi), -720);
  EXPECT_DOUBLE_EQ(rad2deg(0), 0);
  EXPECT_DOUBLE_EQ(rad2deg(pi / 6), 30);
  EXPECT_DOUBLE_EQ(rad2deg(pi / 3), 60);
  EXPECT_DOUBLE_EQ(rad2deg(pi / 2), 90);
  EXPECT_DOUBLE_EQ(rad2deg(pi), 180);
  EXPECT_DOUBLE_EQ(rad2deg(2 * pi), 360);
}

TEST(unit_conversion, kmph2mps)
{
  using autoware::universe_utils::kmph2mps;

  EXPECT_DOUBLE_EQ(kmph2mps(0), 0);
  EXPECT_DOUBLE_EQ(kmph2mps(36), 10);
  EXPECT_DOUBLE_EQ(kmph2mps(72), 20);
  EXPECT_DOUBLE_EQ(kmph2mps(180), 50);
}

TEST(unit_conversion, mps2kmph)
{
  using autoware::universe_utils::mps2kmph;

  EXPECT_DOUBLE_EQ(mps2kmph(0), 0);
  EXPECT_DOUBLE_EQ(mps2kmph(10), 36);
  EXPECT_DOUBLE_EQ(mps2kmph(20), 72);
  EXPECT_DOUBLE_EQ(mps2kmph(50), 180);
}
