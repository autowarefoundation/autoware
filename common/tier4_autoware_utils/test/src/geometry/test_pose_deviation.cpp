// Copyright 2020 Tier IV, Inc.
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

#include "tier4_autoware_utils/geometry/pose_deviation.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <gtest/gtest.h>

TEST(geometry, pose_deviation)
{
  using tier4_autoware_utils::calcPoseDeviation;
  using tier4_autoware_utils::createQuaternionFromRPY;
  using tier4_autoware_utils::deg2rad;

  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 1.0;
  pose1.position.y = 2.0;
  pose1.position.z = 3.0;
  pose1.orientation = createQuaternionFromRPY(0, 0, deg2rad(45));

  geometry_msgs::msg::Pose pose2;
  pose2.position.x = 2.0;
  pose2.position.y = 4.0;
  pose2.position.z = 6.0;
  pose2.orientation = createQuaternionFromRPY(0, 0, deg2rad(60));

  const auto deviation = calcPoseDeviation(pose1, pose2);
  EXPECT_DOUBLE_EQ(deviation.lateral, 0.70710678118654735);
  EXPECT_DOUBLE_EQ(deviation.longitudinal, 2.1213203435596428);
  EXPECT_DOUBLE_EQ(deviation.yaw, deg2rad(15));
}
