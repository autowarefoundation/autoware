// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>

#include "common/types.hpp"
#include "gtest/gtest.h"
#include "trajectory_follower/mpc_trajectory.hpp"

using autoware::common::types::float64_t;
TEST(TestMPCTrajectory, Nominal) {
  typedef autoware::motion::control::trajectory_follower::MPCTrajectory MPCTrajectory;

  MPCTrajectory traj;
  EXPECT_EQ(traj.size(), size_t(0));
  traj.clear();
  EXPECT_EQ(traj.size(), size_t(0));

  traj.push_back(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0);
  ASSERT_EQ(traj.size(), size_t(1));
  EXPECT_EQ(traj.x[0], 0.0);
  EXPECT_EQ(traj.y[0], 1.0);
  EXPECT_EQ(traj.z[0], 2.0);
  EXPECT_EQ(traj.yaw[0], 3.0);
  EXPECT_EQ(traj.vx[0], 4.0);
  EXPECT_EQ(traj.k[0], 5.0);
  EXPECT_EQ(traj.smooth_k[0], 6.0);
  EXPECT_EQ(traj.relative_time[0], 7.0);

  for (float64_t i = 1; i < 11.0; ++i) {
    traj.push_back(i + 0.0, i + 1.0, i + 2.0, i + 3.0, i + 4.0, i + 5.0, i + 6.0, i + 7.0);
  }
  ASSERT_EQ(traj.size(), size_t(11));
  for (size_t i = 0; i < size_t(11); ++i) {
    const float64_t j = static_cast<float64_t>(i);
    EXPECT_EQ(traj.x[i], j);
    EXPECT_EQ(traj.y[i], j + 1.0);
    EXPECT_EQ(traj.z[i], j + 2.0);
    EXPECT_EQ(traj.yaw[i], j + 3.0);
    EXPECT_EQ(traj.vx[i], j + 4.0);
    EXPECT_EQ(traj.k[i], j + 5.0);
    EXPECT_EQ(traj.smooth_k[i], j + 6.0);
    EXPECT_EQ(traj.relative_time[i], j + 7.0);
  }

  traj.clear();
  EXPECT_EQ(traj.size(), size_t(0));
}
