// Copyright 2023 The Autoware Contributors
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

#include "component_interface_specs/control.hpp"
#include "gtest/gtest.h"

TEST(control, interface)
{
  {
    using control_interface::IsPaused;
    IsPaused is_paused;
    size_t depth = 1;
    EXPECT_EQ(is_paused.depth, depth);
    EXPECT_EQ(is_paused.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_paused.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using control_interface::IsStartRequested;
    IsStartRequested is_start_requested;
    size_t depth = 1;
    EXPECT_EQ(is_start_requested.depth, depth);
    EXPECT_EQ(is_start_requested.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_start_requested.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }

  {
    using control_interface::IsStopped;
    IsStopped is_stopped;
    size_t depth = 1;
    EXPECT_EQ(is_stopped.depth, depth);
    EXPECT_EQ(is_stopped.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(is_stopped.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  }
}
