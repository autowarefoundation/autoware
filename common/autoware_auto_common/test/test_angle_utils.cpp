// Copyright 2021 Apex.AI, Inc.
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
//
// Developed by Apex.AI, Inc.

#include <common/types.hpp>
#include <helper_functions/angle_utils.hpp>

#include <gtest/gtest.h>

namespace
{
using autoware::common::helper_functions::wrap_angle;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

}  // namespace

/// @test       Wrap an angle.
TEST(TestAngleUtils, WrapAngle)
{
  EXPECT_DOUBLE_EQ(wrap_angle(-5.0 * M_PI_2), -M_PI_2);
  EXPECT_DOUBLE_EQ(wrap_angle(5.0 * M_PI_2), M_PI_2);
  EXPECT_DOUBLE_EQ(wrap_angle(M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(wrap_angle(-M_PI), -M_PI);
  EXPECT_DOUBLE_EQ(wrap_angle(0.0), 0.0);
}
