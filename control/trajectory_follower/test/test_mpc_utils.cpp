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


#include <memory>
#include <vector>

#include "trajectory_follower/mpc_utils.hpp"
#include "trajectory_follower/mpc_trajectory.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gtest/gtest.h"

namespace
{
namespace MPCUtils = ::autoware::motion::control::trajectory_follower::MPCUtils;
typedef autoware_auto_planning_msgs::msg::Trajectory Trajectory;
typedef autoware_auto_planning_msgs::msg::TrajectoryPoint TrajectoryPoint;
typedef geometry_msgs::msg::Pose Pose;

TEST(TestMPCUtils, CalcNearestIndex) {
  Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  Trajectory trajectory;
  TrajectoryPoint p;
  p.pose.position.x = -2.0;
  p.pose.position.y = 1.0;
  trajectory.points.push_back(p);
  p.pose.position.x = -1.0;
  p.pose.position.y = 1.0;
  trajectory.points.push_back(p);
  p.pose.position.x = 0.0;
  p.pose.position.y = 1.0;
  trajectory.points.push_back(p);
  p.pose.position.x = 1.0;
  p.pose.position.y = 1.0;
  trajectory.points.push_back(p);
  EXPECT_EQ(MPCUtils::calcNearestIndex(trajectory, pose), 2);
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, CalcStopDistance) {
  using autoware_auto_planning_msgs::msg::Trajectory;
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;

  Trajectory trajectory_msg;
  TrajectoryPoint p;
  // Point 0
  p.pose.position.x = 0.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 1
  p.pose.position.x = 1.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 2 - STOP
  p.pose.position.x = 2.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);
  // Point 3
  p.pose.position.x = 3.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 4
  p.pose.position.x = 4.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 5
  p.pose.position.x = 5.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 1.0f;
  trajectory_msg.points.push_back(p);
  // Point 6 - STOP
  p.pose.position.x = 6.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);
  // Point 7 - STOP
  p.pose.position.x = 7.0;
  p.pose.position.y = 0.0;
  p.longitudinal_velocity_mps = 0.0f;
  trajectory_msg.points.push_back(p);

  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 0), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 1), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 2), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 3), 3.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 4), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 5), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 6), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 7), -1.0);
}
}  // namespace
