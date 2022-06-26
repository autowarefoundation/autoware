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
#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_utils.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using behavior_path_planner::isOnRight;
using behavior_path_planner::isSameDirectionShift;
using behavior_path_planner::ObjectData;

TEST(BehaviorPathPlanningAvoidanceUtilsTest, shiftLengthDirectionTest)
{
  ObjectData right_obj;
  right_obj.lateral = -0.3;
  const double negative_shift_length = -1.0;
  const double positive_shift_length = 1.0;

  ASSERT_TRUE(isSameDirectionShift(isOnRight(right_obj), negative_shift_length));
  ASSERT_FALSE(isSameDirectionShift(isOnRight(right_obj), positive_shift_length));

  ObjectData left_obj;
  left_obj.lateral = 0.3;
  ASSERT_TRUE(isSameDirectionShift(isOnRight(left_obj), positive_shift_length));
  ASSERT_FALSE(isSameDirectionShift(isOnRight(left_obj), negative_shift_length));

  const double zero_shift_length = 0.0;
  ASSERT_TRUE(isSameDirectionShift(isOnRight(left_obj), zero_shift_length));
  ASSERT_FALSE(isSameDirectionShift(isOnRight(right_obj), zero_shift_length));
}
