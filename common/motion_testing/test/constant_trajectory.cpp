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
#include <time_utils/time_utils.hpp>

using motion::motion_testing::State;
using motion::motion_testing::Trajectory;
using time_utils::from_message;

TEST(ConstantTrajectory, Stationary)
{
  const auto x0 = 3.0F;
  const auto y0 = -5.0F;
  const auto traj = motion::motion_testing::constant_velocity_trajectory(
    x0, y0, 0.0F, 0.0F, std::chrono::milliseconds(100LL));
  for (auto i = 0U; i < traj.points.size(); ++i) {
    const auto & t = traj.points[i];
    EXPECT_DOUBLE_EQ(t.pose.position.x, x0);
    EXPECT_DOUBLE_EQ(t.pose.position.y, y0);
    EXPECT_DOUBLE_EQ(t.pose.orientation.w, 1.0F);
    EXPECT_DOUBLE_EQ(t.pose.orientation.x, 0.0F);
    EXPECT_DOUBLE_EQ(t.pose.orientation.y, 0.0F);
    EXPECT_DOUBLE_EQ(t.pose.orientation.z, 0.0F);
    EXPECT_FLOAT_EQ(t.longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(t.lateral_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(t.acceleration_mps2, 0.0F);
    EXPECT_FLOAT_EQ(t.heading_rate_rps, 0.0F);
    // TODO(c.ho) check time
    if (i > 0U) {
      const auto dt_curr = from_message(t.time_from_start);
      const auto dt_last = from_message(traj.points[i - 1U].time_from_start);
      EXPECT_GT(dt_curr, dt_last);
    }
  }
}

TEST(ConstantTrajectory, ConstantVelocity)
{
  const auto x0 = 3.0F;
  const auto y0 = -5.0F;
  const auto v0 = 2.0F;
  const auto heading = 30.0F * (3.14159F / 180.0F);
  const auto w = std::cos(heading / 2.0F);
  const auto z = std::sin(heading / 2.0F);
  const auto c = (w + z) * (w - z);
  const auto s = 2.0F * w * z;
  EXPECT_FLOAT_EQ(std::sin(heading), s);
  EXPECT_FLOAT_EQ(std::cos(heading), c);
  const auto dt = std::chrono::milliseconds(100LL);
  const auto traj = motion::motion_testing::constant_velocity_trajectory(
    x0, y0, heading, v0, dt);
  const auto dt_s = std::chrono::duration_cast<std::chrono::duration<float_t>>(dt).count();
  for (auto i = 0U; i < traj.points.size(); ++i) {
    const auto & t = traj.points[i];
    const auto ds = v0 * static_cast<float_t>(i) * dt_s;
    constexpr auto TOL = 1.0E-4F;
    EXPECT_LT(std::fabs(static_cast<float>(t.pose.position.x) - (x0 + (ds * c))), TOL);
    EXPECT_LT(std::fabs(static_cast<float>(t.pose.position.y) - (y0 + (ds * s))), TOL);
    EXPECT_LT(std::fabs(static_cast<float>(t.pose.orientation.w) - (w)), TOL);
    EXPECT_LT(std::fabs(static_cast<float>(t.pose.orientation.z) - (z)), TOL);
    EXPECT_LT(std::fabs(t.longitudinal_velocity_mps - (v0)), TOL);
    EXPECT_LT(std::fabs(t.lateral_velocity_mps - (0.0F)), TOL);
    EXPECT_LT(std::fabs(t.acceleration_mps2 - (0.0F)), TOL);
    EXPECT_LT(std::fabs(t.heading_rate_rps - (0.0F)), TOL);
    // TODO(c.ho) check time
    if (i > 0U) {
      const auto dt_curr = from_message(t.time_from_start);
      const auto dt_last = from_message(traj.points[i - 1U].time_from_start);
      EXPECT_GT(dt_curr, dt_last);
    }
  }
}

TEST(ConstantTrajectory, ConstantAcceleration)
{
  const auto x0 = 3.0F;
  const auto y0 = -5.0F;
  const auto v0 = 0.0F;
  const auto a0 = 1.0F;
  const auto heading = -45.0F * (3.14159F / 180.0F);
  const auto w = std::cos(heading / 2.0F);
  const auto z = std::sin(heading / 2.0F);
  const auto c = (w + z) * (w - z);
  const auto s = 2.0F * w * z;
  EXPECT_FLOAT_EQ(std::sin(heading), s);
  EXPECT_FLOAT_EQ(std::cos(heading), c);
  const auto dt = std::chrono::milliseconds(100LL);
  const auto traj = motion::motion_testing::constant_acceleration_trajectory(
    x0, y0, heading, v0, a0, dt);
  const auto dt_s = std::chrono::duration_cast<std::chrono::duration<float_t>>(dt).count();
  for (auto i = 0U; i < traj.points.size(); ++i) {
    const auto & t = traj.points[i];
    const auto dT = static_cast<float_t>(i) * dt_s;
    const auto ds = dT * (v0 + (0.5F * dT * a0));
    constexpr auto TOL = 1.0E-4F;
    EXPECT_LT(std::abs(static_cast<float>(t.pose.position.x) - (x0 + (ds * c))), TOL);
    EXPECT_LT(std::abs(static_cast<float>(t.pose.position.y) - (y0 + (ds * s))), TOL);
    EXPECT_LT(std::abs(static_cast<float>(t.pose.orientation.w) - (w)), TOL);
    EXPECT_LT(std::abs(static_cast<float>(t.pose.orientation.z) - (z)), TOL);
    EXPECT_LT(std::abs(t.longitudinal_velocity_mps - (v0 + (dT * a0))), TOL);
    EXPECT_LT(std::abs(t.lateral_velocity_mps - (0.0F)), TOL);
    EXPECT_LT(std::abs(t.acceleration_mps2 - (a0)), TOL);
    EXPECT_LT(std::abs(t.heading_rate_rps - (0.0F)), TOL);
    // TODO(c.ho) check time
    if (i > 0U) {
      const auto dt_curr = from_message(t.time_from_start);
      const auto dt_last = from_message(traj.points[i - 1U].time_from_start);
      EXPECT_GT(dt_curr, dt_last);
    }
  }
}

// TODO(c.ho) turn rate tests...
