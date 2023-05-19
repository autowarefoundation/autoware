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

#include "gtest/gtest.h"
#include "motion_utils/distance/distance.hpp"
namespace
{
using motion_utils::calcDecelDistWithJerkAndAccConstraints;

constexpr double epsilon = 1e-3;

TEST(distance, calcDecelDistWithJerkAndAccConstraints)
{
  // invalid velocity
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 20.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);

    EXPECT_FALSE(dist);
  }

  // normal stop
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 0.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    constexpr double expected_dist = 287.224;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // sudden stop
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 0.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -2.5;
    constexpr double jerk_acc = 1.5;
    constexpr double jerk_dec = -1.5;

    constexpr double expected_dist = 69.6947;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // normal deceleration
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 10.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    constexpr double expected_dist = 189.724;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // sudden deceleration
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 10.0;
    constexpr double current_acc = 0.0;
    constexpr double acc_min = -2.5;
    constexpr double jerk_acc = 1.5;
    constexpr double jerk_dec = -1.5;

    constexpr double expected_dist = 58.028;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }

  // current_acc is lower than acc_min
  {
    constexpr double current_vel = 16.7;
    constexpr double target_vel = 0.0;
    constexpr double current_acc = -2.5;
    constexpr double acc_min = -0.5;
    constexpr double jerk_acc = 1.0;
    constexpr double jerk_dec = -0.5;

    constexpr double expected_dist = 217.429;
    const auto dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec);
    EXPECT_NEAR(expected_dist, *dist, epsilon);
  }
}

}  // namespace
