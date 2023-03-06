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

#include "tier4_autoware_utils/math/constants.hpp"
#include "tier4_autoware_utils/math/trigonometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(trigonometry, sin)
{
  float x = 4.f * tier4_autoware_utils::pi / 128.f;
  for (int i = 0; i < 128; i++) {
    EXPECT_TRUE(
      std::abs(
        std::sin(x * static_cast<float>(i)) -
        tier4_autoware_utils::sin(x * static_cast<float>(i))) < 10e-5);
  }
}

TEST(trigonometry, cos)
{
  float x = 4.f * tier4_autoware_utils::pi / 128.f;
  for (int i = 0; i < 128; i++) {
    EXPECT_TRUE(
      std::abs(
        std::cos(x * static_cast<float>(i)) -
        tier4_autoware_utils::cos(x * static_cast<float>(i))) < 10e-5);
  }
}
