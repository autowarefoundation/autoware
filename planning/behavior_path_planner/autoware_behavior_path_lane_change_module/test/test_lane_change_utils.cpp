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
#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

constexpr double epsilon = 1e-6;

TEST(BehaviorPathPlanningLaneChangeUtilsTest, projectCurrentPoseToTarget)
{
  geometry_msgs::msg::Pose ego_pose;
  const auto ego_yaw = autoware::universe_utils::deg2rad(0.0);
  ego_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(ego_yaw);
  ego_pose.position = autoware::universe_utils::createPoint(0, 0, 0);

  geometry_msgs::msg::Pose obj_pose;
  const auto obj_yaw = autoware::universe_utils::deg2rad(0.0);
  obj_pose.orientation = autoware::universe_utils::createQuaternionFromYaw(obj_yaw);
  obj_pose.position = autoware::universe_utils::createPoint(-4, 3, 0);

  const auto result = autoware::universe_utils::inverseTransformPose(obj_pose, ego_pose);

  EXPECT_NEAR(result.position.x, -4, epsilon);
  EXPECT_NEAR(result.position.y, 3, epsilon);
}

TEST(BehaviorPathPlanningLaneChangeUtilsTest, TESTLateralAccelerationMap)
{
  autoware::behavior_path_planner::LateralAccelerationMap lat_acc_map;
  lat_acc_map.add(0.0, 0.2, 0.315);
  lat_acc_map.add(3.0, 0.2, 0.315);
  lat_acc_map.add(5.0, 0.2, 0.315);
  lat_acc_map.add(6.0, 0.315, 0.40);
  lat_acc_map.add(10.0, 0.315, 0.50);

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(-1.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(0.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(1.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(3.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(5.0);
    EXPECT_NEAR(min_acc, 0.2, epsilon);
    EXPECT_NEAR(max_acc, 0.315, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(5.5);
    EXPECT_NEAR(min_acc, 0.2575, epsilon);
    EXPECT_NEAR(max_acc, 0.3575, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(6.0);
    EXPECT_NEAR(min_acc, 0.315, epsilon);
    EXPECT_NEAR(max_acc, 0.4, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(8.0);
    EXPECT_NEAR(min_acc, 0.315, epsilon);
    EXPECT_NEAR(max_acc, 0.45, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(10.0);
    EXPECT_NEAR(min_acc, 0.315, epsilon);
    EXPECT_NEAR(max_acc, 0.50, epsilon);
  }

  {
    const auto [min_acc, max_acc] = lat_acc_map.find(11.0);
    EXPECT_NEAR(min_acc, 0.315, epsilon);
    EXPECT_NEAR(max_acc, 0.50, epsilon);
  }
}
