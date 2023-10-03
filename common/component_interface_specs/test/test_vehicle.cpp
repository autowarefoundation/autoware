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

#include "component_interface_specs/vehicle.hpp"
#include "gtest/gtest.h"

TEST(vehicle, interface)
{
  {
    using vehicle_interface::SteeringStatus;
    SteeringStatus status;
    size_t depth = 1;
    EXPECT_EQ(status.depth, depth);
    EXPECT_EQ(status.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(status.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using vehicle_interface::GearStatus;
    GearStatus status;
    size_t depth = 1;
    EXPECT_EQ(status.depth, depth);
    EXPECT_EQ(status.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(status.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using vehicle_interface::TurnIndicatorStatus;
    TurnIndicatorStatus status;
    size_t depth = 1;
    EXPECT_EQ(status.depth, depth);
    EXPECT_EQ(status.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(status.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using vehicle_interface::HazardLightStatus;
    HazardLightStatus status;
    size_t depth = 1;
    EXPECT_EQ(status.depth, depth);
    EXPECT_EQ(status.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(status.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }

  {
    using vehicle_interface::EnergyStatus;
    EnergyStatus status;
    size_t depth = 1;
    EXPECT_EQ(status.depth, depth);
    EXPECT_EQ(status.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(status.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
  }
}
