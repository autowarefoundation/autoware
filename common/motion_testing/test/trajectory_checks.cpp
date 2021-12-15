// Copyright 2019 Christopher Ho
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
#include <gtest/gtest.h>
#include <motion_testing/motion_testing.hpp>

#include <chrono>

using motion::motion_testing::State;
using motion::motion_testing::Trajectory;
using motion::motion_testing::make_state;
using motion::motion_testing::constant_velocity_trajectory;

TEST(TrajectoryChecks, Basic)
{
  Trajectory traj{};
  const auto target =
    make_state(-100.0F, 100.0F, 2.0F, 1.0F, 0.0F, 0.0F, std::chrono::system_clock::now());
  using motion::motion_testing::progresses_towards_target;
  using motion::motion_testing::dynamically_feasible;
  // Empty case
  ASSERT_TRUE(traj.points.empty());
  EXPECT_EQ(dynamically_feasible(traj), {});
  EXPECT_EQ(progresses_towards_target(traj, target.state), {});
  // Normal case
  traj = constant_velocity_trajectory(1.0F, 1.0F, 2.0F, 3.0F, std::chrono::milliseconds{100LL});
  ASSERT_FALSE(traj.points.empty());
  EXPECT_EQ(dynamically_feasible(traj), traj.points.size());
  EXPECT_EQ(progresses_towards_target(traj, target.state), traj.points.size());
}
