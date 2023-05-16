// Copyright 2023 Tier IV, Inc.
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

#include <sampler_common/structures.hpp>

#include <gtest/gtest.h>

TEST(Path, extendPath)
{
  using sampler_common::Path;
  Path traj1;
  Path traj2;
  Path traj3 = traj1.extend(traj2);
  EXPECT_TRUE(traj3.points.empty());

  traj2.points = {{0, 0}, {1, 1}};
  traj3 = traj1.extend(traj2);
  for (size_t i = 0; i < traj1.points.size(); ++i) {
    EXPECT_EQ(traj3.points[i].x(), traj2.points[i].x());
    EXPECT_EQ(traj3.points[i].y(), traj2.points[i].y());
  }

  traj2.points = {{2, 2}, {3, 3}};
  traj3 = traj3.extend(traj2);
  ASSERT_EQ(traj3.points.size(), 4ul);
  for (size_t i = 0; i < traj1.points.size(); ++i) {
    EXPECT_EQ(traj3.points[i].x(), i);
    EXPECT_EQ(traj3.points[i].y(), i);
  }
}

TEST(Trajectory, resample)
{
  constexpr auto eps = 1e-6;
  using sampler_common::Trajectory;

  Trajectory t;
  t.reserve(2);
  t.points = {{0, 0}, {1, 1}};
  t.yaws = {0.0, M_PI};
  t.lengths = {0.0, 1.0};
  t.times = {0.0, 1.0};
  t.jerks = {0.0, 0.0};
  t.longitudinal_velocities = {0.0, 0.5};
  t.longitudinal_accelerations = {0.0, 1.0};
  t.lateral_velocities = {0.0, 1.5};
  t.lateral_accelerations = {0.0, 3.0};
  t.curvatures = {0.0, 0.9};

  Trajectory t2 = t.resample(0.5);
  ASSERT_EQ(t2.points.size(), 3lu);
  EXPECT_NEAR(t2.points[0].x(), 0.0, eps);
  EXPECT_NEAR(t2.points[1].x(), 0.5, eps);
  EXPECT_NEAR(t2.points[2].x(), 1.0, eps);
  EXPECT_NEAR(t2.points[0].y(), 0.0, eps);
  EXPECT_NEAR(t2.points[1].y(), 0.5, eps);
  EXPECT_NEAR(t2.points[2].y(), 1.0, eps);
  ASSERT_EQ(t2.yaws.size(), 3lu);
  EXPECT_NEAR(t2.yaws[0], 0.0, eps);
  EXPECT_NEAR(t2.yaws[1], M_PI_2, eps);
  EXPECT_NEAR(t2.yaws[2], M_PI, eps);
  ASSERT_EQ(t2.lengths.size(), 3lu);
  EXPECT_NEAR(t2.lengths[0], 0.0, eps);
  EXPECT_NEAR(t2.lengths[1], 0.5, eps);
  EXPECT_NEAR(t2.lengths[2], 1.0, eps);
  ASSERT_EQ(t2.times.size(), 3lu);
  EXPECT_NEAR(t2.times[0], 0.0, eps);
  EXPECT_NEAR(t2.times[1], 0.5, eps);
  EXPECT_NEAR(t2.times[2], 1.0, eps);
  ASSERT_EQ(t2.jerks.size(), 3lu);
  EXPECT_NEAR(t2.jerks[0], 0.0, eps);
  EXPECT_NEAR(t2.jerks[1], 0.0, eps);
  EXPECT_NEAR(t2.jerks[2], 0.0, eps);
  ASSERT_EQ(t2.longitudinal_velocities.size(), 3lu);
  EXPECT_NEAR(t2.longitudinal_velocities[0], 0.0, eps);
  EXPECT_NEAR(t2.longitudinal_velocities[1], 0.25, eps);
  EXPECT_NEAR(t2.longitudinal_velocities[2], 0.5, eps);
  ASSERT_EQ(t2.longitudinal_accelerations.size(), 3lu);
  EXPECT_NEAR(t2.longitudinal_accelerations[0], 0.0, eps);
  EXPECT_NEAR(t2.longitudinal_accelerations[1], 0.5, eps);
  EXPECT_NEAR(t2.longitudinal_accelerations[2], 1.0, eps);
  ASSERT_EQ(t2.lateral_velocities.size(), 3lu);
  EXPECT_NEAR(t2.lateral_velocities[0], 0.0, eps);
  EXPECT_NEAR(t2.lateral_velocities[1], 0.75, eps);
  EXPECT_NEAR(t2.lateral_velocities[2], 1.5, eps);
  ASSERT_EQ(t2.lateral_accelerations.size(), 3lu);
  EXPECT_NEAR(t2.lateral_accelerations[0], 0.0, eps);
  EXPECT_NEAR(t2.lateral_accelerations[1], 1.5, eps);
  EXPECT_NEAR(t2.lateral_accelerations[2], 3.0, eps);
}

TEST(Trajectory, resampleTime)
{
  constexpr auto eps = 1e-6;
  using sampler_common::Trajectory;

  Trajectory t;
  t.reserve(2);
  t.points = {{0, 0}, {1, 1}};
  t.yaws = {0.0, M_PI};
  t.lengths = {0.0, 1.0};
  t.times = {0.0, 1.0};
  t.jerks = {0.0, 0.0};
  t.longitudinal_velocities = {0.0, 0.5};
  t.longitudinal_accelerations = {0.0, 1.0};
  t.lateral_velocities = {0.0, 1.5};
  t.lateral_accelerations = {0.0, 3.0};
  t.curvatures = {0.0, 0.9};

  Trajectory t2 = t.resampleTimeFromZero(0.5);
  ASSERT_EQ(t2.points.size(), 3lu);
  EXPECT_NEAR(t2.points[0].x(), 0.0, eps);
  EXPECT_NEAR(t2.points[1].x(), 0.5, eps);
  EXPECT_NEAR(t2.points[2].x(), 1.0, eps);
  EXPECT_NEAR(t2.points[0].y(), 0.0, eps);
  EXPECT_NEAR(t2.points[1].y(), 0.5, eps);
  EXPECT_NEAR(t2.points[2].y(), 1.0, eps);
  ASSERT_EQ(t2.yaws.size(), 3lu);
  EXPECT_NEAR(t2.yaws[0], 0.0, eps);
  EXPECT_NEAR(t2.yaws[1], M_PI_2, eps);
  EXPECT_NEAR(t2.yaws[2], M_PI, eps);
  ASSERT_EQ(t2.lengths.size(), 3lu);
  EXPECT_NEAR(t2.lengths[0], 0.0, eps);
  EXPECT_NEAR(t2.lengths[1], 0.5, eps);
  EXPECT_NEAR(t2.lengths[2], 1.0, eps);
  ASSERT_EQ(t2.times.size(), 3lu);
  EXPECT_NEAR(t2.times[0], 0.0, eps);
  EXPECT_NEAR(t2.times[1], 0.5, eps);
  EXPECT_NEAR(t2.times[2], 1.0, eps);
  ASSERT_EQ(t2.jerks.size(), 3lu);
  EXPECT_NEAR(t2.jerks[0], 0.0, eps);
  EXPECT_NEAR(t2.jerks[1], 0.0, eps);
  EXPECT_NEAR(t2.jerks[2], 0.0, eps);
  ASSERT_EQ(t2.longitudinal_velocities.size(), 3lu);
  EXPECT_NEAR(t2.longitudinal_velocities[0], 0.0, eps);
  EXPECT_NEAR(t2.longitudinal_velocities[1], 0.25, eps);
  EXPECT_NEAR(t2.longitudinal_velocities[2], 0.5, eps);
  ASSERT_EQ(t2.longitudinal_accelerations.size(), 3lu);
  EXPECT_NEAR(t2.longitudinal_accelerations[0], 0.0, eps);
  EXPECT_NEAR(t2.longitudinal_accelerations[1], 0.5, eps);
  EXPECT_NEAR(t2.longitudinal_accelerations[2], 1.0, eps);
  ASSERT_EQ(t2.lateral_velocities.size(), 3lu);
  EXPECT_NEAR(t2.lateral_velocities[0], 0.0, eps);
  EXPECT_NEAR(t2.lateral_velocities[1], 0.75, eps);
  EXPECT_NEAR(t2.lateral_velocities[2], 1.5, eps);
  ASSERT_EQ(t2.lateral_accelerations.size(), 3lu);
  EXPECT_NEAR(t2.lateral_accelerations[0], 0.0, eps);
  EXPECT_NEAR(t2.lateral_accelerations[1], 1.5, eps);
  EXPECT_NEAR(t2.lateral_accelerations[2], 3.0, eps);
}
