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

#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/utils.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using autoware::behavior_path_planner::ObjectData;
using autoware::behavior_path_planner::utils::static_obstacle_avoidance::isOnRight;
using autoware::behavior_path_planner::utils::static_obstacle_avoidance::isSameDirectionShift;
using autoware::behavior_path_planner::utils::static_obstacle_avoidance::isShiftNecessary;
using autoware::route_handler::Direction;

TEST(BehaviorPathPlanningAvoidanceUtilsTest, shiftLengthDirectionTest)
{
  ObjectData right_obj;
  right_obj.direction = Direction::RIGHT;
  const double negative_shift_length = -1.0;
  const double positive_shift_length = 1.0;

  ASSERT_TRUE(isSameDirectionShift(isOnRight(right_obj), negative_shift_length));
  ASSERT_FALSE(isSameDirectionShift(isOnRight(right_obj), positive_shift_length));

  ObjectData left_obj;
  left_obj.direction = Direction::LEFT;
  ASSERT_TRUE(isSameDirectionShift(isOnRight(left_obj), positive_shift_length));
  ASSERT_FALSE(isSameDirectionShift(isOnRight(left_obj), negative_shift_length));
}

TEST(BehaviorPathPlanningAvoidanceUtilsTest, shiftNecessaryTest)
{
  ObjectData right_obj;
  right_obj.direction = Direction::RIGHT;
  const double negative_shift_length = -1.0;
  const double positive_shift_length = 1.0;

  ASSERT_TRUE(isShiftNecessary(isOnRight(right_obj), positive_shift_length));
  ASSERT_FALSE(isShiftNecessary(isOnRight(right_obj), negative_shift_length));

  ObjectData left_obj;
  left_obj.direction = Direction::LEFT;
  ASSERT_TRUE(isShiftNecessary(isOnRight(left_obj), negative_shift_length));
  ASSERT_FALSE(isShiftNecessary(isOnRight(left_obj), positive_shift_length));
}
