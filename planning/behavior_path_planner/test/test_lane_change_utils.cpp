// Copyright 2022 TIER IV, Inc.
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
#include "behavior_path_planner/scene_module/lane_change/util.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

TEST(BehaviorPathPlanningLaneChangeUtilsTest, testStoppingDistance)
{
  const auto vehicle_velocity = 8.333;

  const auto negative_accel = -1.5;
  const auto distance_when_negative =
    behavior_path_planner::lane_change_utils::stoppingDistance(vehicle_velocity, negative_accel);
  ASSERT_NEAR(distance_when_negative, 23.1463, 1e-3);

  const auto positive_accel = 1.5;
  const auto distance_when_positive =
    behavior_path_planner::lane_change_utils::stoppingDistance(vehicle_velocity, positive_accel);
  ASSERT_NEAR(distance_when_positive, 34.7194, 1e-3);

  const auto zero_accel = 0.0;
  const auto distance_when_zero =
    behavior_path_planner::lane_change_utils::stoppingDistance(vehicle_velocity, zero_accel);
  ASSERT_NEAR(distance_when_zero, 34.7194, 1e-3);
}
