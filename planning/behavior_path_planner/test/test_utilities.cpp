// Copyright 2022 TIER IV, Inc.
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
#include "input.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using behavior_path_planner::PathWithLaneId;
using behavior_path_planner::Pose;
using behavior_path_planner::util::FrenetCoordinate3d;
using geometry_msgs::msg::Point;

TEST(BehaviorPathPlanningUtilitiesBehaviorTest, vehiclePoseToFrenetOnStraightLine)
{
  PathWithLaneId path =
    behavior_path_planner::generateStraightSamplePathWithLaneId(0.0f, 1.0f, 10u);
  std::vector<geometry_msgs::msg::Pose> geometry_points =
    behavior_path_planner::util::convertToPoseArray(path);
  Pose vehicle_pose = behavior_path_planner::generateEgoSamplePose(10.7f, -1.7f, 0.0);

  const size_t vehicle_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    geometry_points, vehicle_pose, 3.0, 1.0);
  FrenetCoordinate3d vehicle_pose_frenet = behavior_path_planner::util::convertToFrenetCoordinate3d(
    geometry_points, vehicle_pose.position, vehicle_seg_idx);

  EXPECT_NEAR(vehicle_pose_frenet.distance, -1.7f, 1e-3);
  EXPECT_NEAR(vehicle_pose_frenet.length, 10.7f, 1e-3);
}

TEST(BehaviorPathPlanningUtilitiesBehaviorTest, vehiclePoseToFrenetOnDiagonalLine)
{
  PathWithLaneId path =
    behavior_path_planner::generateDiagonalSamplePathWithLaneId(0.0f, 1.0f, 10u);
  std::vector<geometry_msgs::msg::Pose> geometry_points =
    behavior_path_planner::util::convertToPoseArray(path);
  Pose vehicle_pose = behavior_path_planner::generateEgoSamplePose(0.1f, 0.1f, 0.0);

  const size_t vehicle_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    geometry_points, vehicle_pose, 3.0, 1.0);
  FrenetCoordinate3d vehicle_pose_frenet = behavior_path_planner::util::convertToFrenetCoordinate3d(
    geometry_points, vehicle_pose.position, vehicle_seg_idx);

  EXPECT_NEAR(vehicle_pose_frenet.distance, 0, 1e-2);
  EXPECT_NEAR(vehicle_pose_frenet.length, 0.1414f, 1e-2);
}

TEST(BehaviorPathPlanningUtilitiesBehaviorTest, setGoal)
{
  PathWithLaneId path = behavior_path_planner::generateDiagonalSamplePathWithLaneId(0.0f, 1.0f, 5u);
  path.points.at(0).lane_ids.push_back(0);
  path.points.at(1).lane_ids.push_back(1);
  path.points.at(2).lane_ids.push_back(2);
  path.points.at(3).lane_ids.push_back(3);
  path.points.at(3).lane_ids.push_back(4);
  path.points.at(3).lane_ids.push_back(5);
  path.points.at(4).lane_ids.push_back(5);

  PathWithLaneId path_with_goal;
  behavior_path_planner::util::setGoal(
    3.5, M_PI * 0.5, path, path.points.back().point.pose, 5, &path_with_goal);

  // Check if skipped lane ids by smooth skip connection are filled in output path.
  EXPECT_EQ(path_with_goal.points.size(), 4U);
  ASSERT_THAT(path_with_goal.points.at(0).lane_ids, testing::ElementsAre(0));
  ASSERT_THAT(path_with_goal.points.at(1).lane_ids, testing::ElementsAre(1));
  ASSERT_THAT(path_with_goal.points.at(2).lane_ids, testing::ElementsAre(2, 3, 4, 5));
  ASSERT_THAT(path_with_goal.points.at(3).lane_ids, testing::ElementsAre(5));
}
