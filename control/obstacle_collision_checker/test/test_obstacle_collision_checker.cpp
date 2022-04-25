// Copyright 2022 Tier IV, Inc. All rights reserved.
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

#include "../src/obstacle_collision_checker_node/obstacle_collision_checker.cpp"  // NOLINT
#include "gtest/gtest.h"

TEST(test_obstacle_collision_checker, filterPointCloudByTrajectory)
{
  pcl::PointCloud<pcl::PointXYZ> pcl;
  autoware_auto_planning_msgs::msg::Trajectory trajectory;
  pcl::PointXYZ pcl_point;
  autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
  pcl_point.y = 0.0;
  traj_point.pose.position.y = 0.99;
  for (double x = 0.0; x < 10.0; x += 1.0) {
    pcl_point.x = x;
    traj_point.pose.position.x = x;
    trajectory.points.push_back(traj_point);
    pcl.push_back(pcl_point);
  }
  // radius < 1: all points are filtered
  for (auto radius = 0.0; radius <= 0.99; radius += 0.1) {
    const auto filtered_pcl = filterPointCloudByTrajectory(pcl, trajectory, radius);
    EXPECT_EQ(filtered_pcl.size(), 0ul);
  }
  // radius >= 1.0: all points are kept
  for (auto radius = 1.0; radius < 10.0; radius += 0.1) {
    const auto filtered_pcl = filterPointCloudByTrajectory(pcl, trajectory, radius);
    ASSERT_EQ(pcl.size(), filtered_pcl.size());
    for (size_t i = 0; i < pcl.size(); ++i) {
      EXPECT_EQ(pcl[i].x, filtered_pcl[i].x);
      EXPECT_EQ(pcl[i].y, filtered_pcl[i].y);
    }
  }
}
