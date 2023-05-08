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
#include "behavior_path_planner/utils/safety_check.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

constexpr double epsilon = 1e-6;

TEST(BehaviorPathPlanningLaneChangeUtilsTest, projectCurrentPoseToTarget)
{
  geometry_msgs::msg::Pose ego_pose;
  const auto ego_yaw = tier4_autoware_utils::deg2rad(0.0);
  ego_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(ego_yaw);
  ego_pose.position = tier4_autoware_utils::createPoint(0, 0, 0);

  geometry_msgs::msg::Pose obj_pose;
  const auto obj_yaw = tier4_autoware_utils::deg2rad(0.0);
  obj_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(obj_yaw);
  obj_pose.position = tier4_autoware_utils::createPoint(-4, 3, 0);

  const auto result = tier4_autoware_utils::inverseTransformPose(obj_pose, ego_pose);

  EXPECT_NEAR(result.position.x, -4, epsilon);
  EXPECT_NEAR(result.position.y, 3, epsilon);
}
