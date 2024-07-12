// Copyright 2024 The Autoware Foundation
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

#include "unit_conversions.hpp"

#include <gtest/gtest.h>

namespace autoware::component_monitor::unit_conversions
{
TEST(UnitConversions, kib_to_bytes)
{
  EXPECT_EQ(kib_to_bytes(1), 1024U);
  EXPECT_EQ(kib_to_bytes(0), 0U);
  EXPECT_EQ(kib_to_bytes(10), 10240U);
}
TEST(UnitConversions, mib_to_bytes)
{
  EXPECT_EQ(mib_to_bytes(1), 1048576U);
  EXPECT_EQ(mib_to_bytes(0), 0U);
  EXPECT_EQ(mib_to_bytes(10), 10485760U);
}
TEST(UnitConversions, gib_to_bytes)
{
  EXPECT_EQ(gib_to_bytes(1), 1073741824U);
  EXPECT_EQ(gib_to_bytes(0), 0U);
  EXPECT_EQ(gib_to_bytes(10), 10737418240U);
}
TEST(UnitConversions, tib_to_bytes)
{
  EXPECT_EQ(tib_to_bytes(1), 1099511627776U);
  EXPECT_EQ(tib_to_bytes(0), 0U);
  EXPECT_EQ(tib_to_bytes(10), 10995116277760U);
}
TEST(UnitConversions, pib_to_bytes)
{
  EXPECT_EQ(pib_to_bytes(1), 1125899906842624U);
  EXPECT_EQ(pib_to_bytes(0), 0U);
  EXPECT_EQ(pib_to_bytes(10), 11258999068426240U);
}
TEST(UnitConversions, eib_to_bytes)
{
  EXPECT_EQ(eib_to_bytes(1), 1152921504606846976U);
  EXPECT_EQ(eib_to_bytes(0), 0U);
  EXPECT_EQ(eib_to_bytes(10), 11529215046068469760U);
}
}  // namespace autoware::component_monitor::unit_conversions
