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

#include "gtest/gtest.h"
#include "trajectory_follower/debug_values.hpp"

TEST(TestDebugValues, assign_and_get) {
  using ::autoware::motion::control::trajectory_follower::DebugValues;
  DebugValues debug;

  EXPECT_EQ(debug.getValues().size(), static_cast<size_t>(DebugValues::TYPE::SIZE));
  debug.setValues(DebugValues::TYPE::DT, 42.0);
  EXPECT_EQ(debug.getValues().at(debug.getValuesIdx(DebugValues::TYPE::DT)), 42.0);
  debug.setValues(debug.getValuesIdx(DebugValues::TYPE::DT), 4.0);
  EXPECT_EQ(debug.getValues().at(debug.getValuesIdx(DebugValues::TYPE::DT)), 4.0);
}
