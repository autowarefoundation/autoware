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

#include "motion_utils/trajectory/trajectory.hpp"
#include "utilization/path_utilization.hpp"
#include "utils.hpp"

#include <utilization/boost_geometry_helper.hpp>
#include <utilization/util.hpp>

#include <gtest/gtest.h>

#include <iostream>

#define DEBUG_PRINT_PATH(path)                                                        \
  {                                                                                   \
    std::stringstream ss;                                                             \
    ss << #path << "(px, vx): ";                                                      \
    for (const auto p : path.points) {                                                \
      ss << "(" << p.pose.position.x << ", " << p.longitudinal_velocity_mps << "), "; \
    }                                                                                 \
    std::cerr << ss.str() << std::endl;                                               \
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

TEST(specialInterpolation, specialInterpolation)
{
  using autoware_auto_planning_msgs::msg::Path;
  using autoware_auto_planning_msgs::msg::PathPoint;
  using behavior_velocity_planner::interpolatePath;
  using motion_utils::calcSignedArcLength;
  using motion_utils::searchZeroVelocityIndex;

  const auto genPath = [](const auto p, const auto v) {
    if (p.size() != v.size()) throw std::invalid_argument("different size is not expected");
    Path path;
    for (size_t i = 0; i < p.size(); ++i) {
      PathPoint pp;
      pp.pose.position.x = p.at(i);
      pp.longitudinal_velocity_mps = v.at(i);
      path.points.push_back(pp);
    }
    return path;
  };

  constexpr auto length = 5.0;
  constexpr auto interval = 1.0;

  const auto calcInterpolatedStopDist = [&](const auto & px, const auto & vx) {
    const auto path = genPath(px, vx);
    const auto res = interpolatePath(path, length, interval);
    // DEBUG_PRINT_PATH(path);
    // DEBUG_PRINT_PATH(res);
    return calcSignedArcLength(res.points, 0, *searchZeroVelocityIndex(res.points));
  };

  // expected stop position: s=2.0
  {
    const std::vector<double> px{0.0, 1.0, 2.0, 3.0};
    const std::vector<double> vx{5.5, 5.5, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), 2.0);
  }

  // expected stop position: s=2.1
  {
    constexpr auto expected = 2.1;
    const std::vector<double> px{0.0, 1.0, 2.1, 3.0};
    const std::vector<double> vx{5.5, 5.5, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=2.001
  {
    constexpr auto expected = 2.001;
    const std::vector<double> px{0.0, 1.0, 2.001, 3.0};
    const std::vector<double> vx{5.5, 5.5, 0.000, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=2.001
  {
    constexpr auto expected = 2.001;
    const std::vector<double> px{0.0, 1.0, 1.999, 2.0, 2.001, 3.0};
    const std::vector<double> vx{5.5, 5.5, 5.555, 5.5, 0.000, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=2.0
  {
    constexpr auto expected = 2.0;
    const std::vector<double> px{0.0, 1.0, 1.999, 2.0, 2.001, 3.0};
    const std::vector<double> vx{5.5, 5.5, 5.555, 0.0, 0.000, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=1.999
  {
    constexpr auto expected = 1.999;
    const std::vector<double> px{0.0, 1.0, 1.999, 3.0};
    const std::vector<double> vx{5.5, 5.5, 0.000, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=0.2
  {
    constexpr auto expected = 0.2;
    const std::vector<double> px{0.0, 0.1, 0.2, 0.3, 0.4};
    const std::vector<double> vx{5.5, 5.5, 0.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }

  // expected stop position: s=0.4
  {
    constexpr auto expected = 0.4;
    const std::vector<double> px{0.0, 0.1, 0.2, 0.3, 0.4};
    const std::vector<double> vx{5.5, 5.5, 5.5, 5.5, 0.0};
    EXPECT_DOUBLE_EQ(calcInterpolatedStopDist(px, vx), expected);
  }
}
