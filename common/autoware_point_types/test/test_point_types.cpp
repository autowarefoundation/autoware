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

#include "autoware_point_types/types.hpp"

#include <gtest/gtest.h>

#include <limits>

TEST(PointEquality, PointXYZI)
{
  using autoware_point_types::PointXYZI;

  PointXYZI pt0{0, 1, 2, 3};
  PointXYZI pt1{0, 1, 2, 3};
  EXPECT_EQ(pt0, pt1);
}

TEST(PointEquality, PointXYZIRADRT)
{
  using autoware_point_types::PointXYZIRADRT;

  PointXYZIRADRT pt0{0, 1, 2, 3, 4, 5, 6, 7, 8};
  PointXYZIRADRT pt1{0, 1, 2, 3, 4, 5, 6, 7, 8};
  EXPECT_EQ(pt0, pt1);
}

TEST(PointEquality, FloatEq)
{
  // test template
  EXPECT_TRUE(autoware_point_types::float_eq<float>(1, 1));
  EXPECT_TRUE(autoware_point_types::float_eq<double>(1, 1));

  // test floating point error
  EXPECT_TRUE(autoware_point_types::float_eq<float>(1, 1 + std::numeric_limits<float>::epsilon()));

  // test difference of sign
  EXPECT_FALSE(autoware_point_types::float_eq<float>(2, -2));
  EXPECT_FALSE(autoware_point_types::float_eq<float>(-2, 2));

  // small value difference
  EXPECT_FALSE(autoware_point_types::float_eq<float>(2, 2 + 10e-6));

  // expect same value if epsilon is larger than difference
  EXPECT_TRUE(autoware_point_types::float_eq<float>(2, 2 + 10e-6, 10e-5));
}
