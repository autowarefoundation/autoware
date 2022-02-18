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

#include "utils.hpp"

#include <utilization/boost_geometry_helper.hpp>
#include <utilization/util.hpp>

#include <gtest/gtest.h>

TEST(to_footprint_polygon, nominal)
{
  using behavior_velocity_planner::planning_utils::toFootprintPolygon;
  autoware_auto_perception_msgs::msg::PredictedObject obj = test::generatePredictedObject(0.0);
  auto poly = toFootprintPolygon(obj);
  EXPECT_TRUE(true);
}

TEST(is_ahead_of, nominal)
{
  using behavior_velocity_planner::planning_utils::isAheadOf;
  geometry_msgs::msg::Pose target = test::generatePose(0);
  geometry_msgs::msg::Pose origin = test::generatePose(1);
  bool is_ahead = isAheadOf(target, origin);
  EXPECT_FALSE(is_ahead);
  target = test::generatePose(2);
  is_ahead = isAheadOf(target, origin);
  EXPECT_TRUE(is_ahead);
}

TEST(smoothDeceleration, calculateMaxSlowDownVelocity)
{
  using behavior_velocity_planner::planning_utils::calcDecelerationVelocityFromDistanceToTarget;
  const double current_accel = 1.0;
  const double current_velocity = 5.0;
  const double max_slow_down_jerk = -1.0;
  const double max_slow_down_accel = -2.0;
  const double eps = 1e-3;
  {
    for (int i = -8; i <= 24; i += 8) {
      // arc length in path point
      const double l = i * 1.0;
      const double v = calcDecelerationVelocityFromDistanceToTarget(
        max_slow_down_jerk, max_slow_down_accel, current_accel, current_velocity, l);
      // case 0 : behind ego
      if (i == -8) EXPECT_NEAR(v, 5.0, eps);
      // case 1 : const jerk
      else if (i == 0)
        EXPECT_NEAR(v, 5.0, eps);
      // case 1 : const jerk
      else if (i == 8)
        EXPECT_NEAR(v, 5.380, eps);
      // case 2 : const accel
      else if (i == 16)
        EXPECT_NEAR(v, 2.872, eps);
      // case 3 : after stop
      else if (i == 24)
        EXPECT_NEAR(v, 0.00, eps);
      else
        continue;
      std::cout << "s: " << l << " v: " << v << std::endl;
    }
  }
}
