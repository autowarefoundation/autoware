// Copyright 2022 Tier IV, Inc.
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

#include "autoware/velocity_smoother/trajectory_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

using autoware::velocity_smoother::trajectory_utils::TrajectoryPoints;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoints genStraightTrajectory(const size_t size)
{
  double x = 0.0;
  double dx = 1.0;
  geometry_msgs::msg::Quaternion o;
  o.x = 0.0;
  o.y = 0.0;
  o.z = 0.0;
  o.w = 1.0;

  TrajectoryPoints tps;
  TrajectoryPoint p;
  for (size_t i = 0; i < size; ++i) {
    p.pose.position.x = x;
    p.pose.orientation = o;
    x += dx;
    tps.push_back(p);
  }
  return tps;
}

TEST(TestTrajectoryUtils, CalcTrajectoryCurvatureFrom3Points)
{
  // the output curvature vector should have the same size of the input trajectory.
  const auto checkOutputSize = [](const size_t trajectory_size, const size_t idx_dist) {
    const auto trajectory_points = genStraightTrajectory(trajectory_size);
    const auto curvatures =
      autoware::velocity_smoother::trajectory_utils::calcTrajectoryCurvatureFrom3Points(
        trajectory_points, idx_dist);
    EXPECT_EQ(curvatures.size(), trajectory_size) << ", idx_dist = " << idx_dist;
  };

  const auto trajectory_size_arr = {0, 1, 2, 3, 4, 5, 10, 100};
  const auto idx_dist_arr = {0, 1, 2, 3, 4, 5, 10, 100};
  for (const auto trajectory_size : trajectory_size_arr) {
    for (const auto idx_dist : idx_dist_arr) {
      checkOutputSize(trajectory_size, idx_dist);
    }
  }
}
