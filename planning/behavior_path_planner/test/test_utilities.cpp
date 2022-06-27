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
  std::vector<Point> geometry_points =
    behavior_path_planner::util::convertToGeometryPointArray(path);
  Pose vehicle_pose = behavior_path_planner::generateEgoSamplePose(10.7f, -1.7f, 0.0);

  FrenetCoordinate3d vehicle_pose_frenet = behavior_path_planner::util::convertToFrenetCoordinate3d(
    geometry_points, vehicle_pose.position);

  EXPECT_NEAR(vehicle_pose_frenet.distance, -1.7f, 1e-3);
  EXPECT_NEAR(vehicle_pose_frenet.length, 10.7f, 1e-3);
}

TEST(BehaviorPathPlanningUtilitiesBehaviorTest, vehiclePoseToFrenetOnDiagonalLine)
{
  PathWithLaneId path =
    behavior_path_planner::generateDiagonalSamplePathWithLaneId(0.0f, 1.0f, 10u);
  std::vector<Point> geometry_points =
    behavior_path_planner::util::convertToGeometryPointArray(path);
  Pose vehicle_pose = behavior_path_planner::generateEgoSamplePose(0.1f, 0.1f, 0.0);

  FrenetCoordinate3d vehicle_pose_frenet = behavior_path_planner::util::convertToFrenetCoordinate3d(
    geometry_points, vehicle_pose.position);

  EXPECT_NEAR(vehicle_pose_frenet.distance, 0, 1e-2);
  EXPECT_NEAR(vehicle_pose_frenet.length, 0.1414f, 1e-2);
}
