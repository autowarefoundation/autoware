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

#include <vector>

#include "gtest/gtest.h"
#include "trajectory_follower/pid.hpp"

TEST(TestPID, calculate_pid_output) {
  using ::autoware::motion::control::trajectory_follower::PIDController;
  std::vector<double> contributions;
  const double dt = 1.0;
  double target = 10.0;
  double current = 0.0;
  bool enable_integration = false;

  PIDController pid;

  // Cannot calculate before initializing gains and limits
  EXPECT_THROW(pid.calculate(0.0, dt, enable_integration, contributions), std::runtime_error);

  pid.setGains(1.0, 1.0, 1.0);
  pid.setLimits(10.0, 0.0, 10.0, 0.0, 10.0, 0.0, 10.0, 0.0);
  double error = target - current;
  double prev_error = error;
  while (current != target) {
    current = pid.calculate(error, dt, enable_integration, contributions);
    EXPECT_EQ(contributions[0], error);
    EXPECT_EQ(contributions[1], 0.0);  // integration is deactivated
    EXPECT_EQ(contributions[2], error - prev_error);
    prev_error = error;
    error = target - current;
  }
  pid.reset();

  pid.setGains(100.0, 100.0, 100.0);
  pid.setLimits(10.0, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0, -10.0);
  enable_integration = true;

  // High errors to force each component to its upper limit
  EXPECT_EQ(pid.calculate(0.0, dt, enable_integration, contributions), 0.0);
  for (double error = 100.0; error < 1000.0; error += 100.0) {
    EXPECT_EQ(pid.calculate(error, dt, enable_integration, contributions), 10.0);
    EXPECT_EQ(contributions[0], 10.0);
    EXPECT_EQ(contributions[1], 10.0);  // integration is activated
    EXPECT_EQ(contributions[2], 10.0);
  }

  // Low errors to force each component to its lower limit
  EXPECT_EQ(pid.calculate(0.0, dt, enable_integration, contributions), 0.0);
  for (double error = -100.0; error > -1000.0; error -= 100.0) {
    EXPECT_EQ(pid.calculate(error, dt, enable_integration, contributions), -10.0);
    EXPECT_EQ(contributions[0], -10.0);
    EXPECT_EQ(contributions[1], -10.0);  // integration is activated
    EXPECT_EQ(contributions[2], -10.0);
  }
}
