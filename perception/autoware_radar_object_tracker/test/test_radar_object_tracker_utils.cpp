// Copyright 2024 TIER IV, Inc.
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

#include "autoware_radar_object_tracker/utils/radar_object_tracker_utils.hpp"

#include <gtest/gtest.h>

using autoware::radar_object_tracker::utils::checkCloseLaneletCondition;
using autoware_perception_msgs::msg::TrackedObject;

// helper function to create a dummy straight lanelet
lanelet::Lanelet createDummyStraightLanelet(double length, double width)
{
  lanelet::LineString3d left_line(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 0, 0, 0),
                              lanelet::Point3d(lanelet::utils::getId(), length, 0, 0)});
  lanelet::LineString3d right_line(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 0, width, 0),
                              lanelet::Point3d(lanelet::utils::getId(), length, width, 0)});
  lanelet::LineString3d center_line(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 0, width / 2, 0),
                              lanelet::Point3d(lanelet::utils::getId(), length, width / 2, 0)});

  // Correct constructor call
  return lanelet::Lanelet(
    lanelet::utils::getId(), left_line, right_line, lanelet::AttributeMap(),
    lanelet::RegulatoryElementPtrs());
}

// helper function to create a dummy tracked object
TrackedObject createDummyTrackedObject(double x, double y, double yaw, double velocity)
{
  TrackedObject obj;
  obj.kinematics.pose_with_covariance.pose.position.x = x;
  obj.kinematics.pose_with_covariance.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  obj.kinematics.pose_with_covariance.pose.orientation = tf2::toMsg(q);
  obj.kinematics.twist_with_covariance.twist.linear.x = velocity;
  return obj;
}

// 1. Test checkCloseLaneletCondition
// 1. Inside lanelet
TEST(CheckCloseLaneletConditionTest, InsideLanelet)
{
  auto lanelet = createDummyStraightLanelet(100.0, 3.0);
  TrackedObject obj = createDummyTrackedObject(50.0, 1.5, 0.0, 10.0);

  bool result = checkCloseLaneletCondition({0.0, lanelet}, obj, 5.0, M_PI / 4);
  EXPECT_TRUE(result);
}

// 2. Outside lanelet but close
TEST(CheckCloseLaneletConditionTest, OutsideLaneletButClose)
{
  auto lanelet = createDummyStraightLanelet(100.0, 3.0);
  TrackedObject obj = createDummyTrackedObject(5.0, 50.0, 0.0, 10.0);

  bool result = checkCloseLaneletCondition({3.0, lanelet}, obj, 5.0, M_PI / 4);
  EXPECT_TRUE(result);
}

// 3. Outside lanelet and far
TEST(CheckCloseLaneletConditionTest, OutsideLaneletAndFar)
{
  auto lanelet = createDummyStraightLanelet(100.0, 3.0);
  TrackedObject obj = createDummyTrackedObject(10.0, 50.0, 0.0, 10.0);

  bool result = checkCloseLaneletCondition({10.0, lanelet}, obj, 5.0, M_PI / 4);
  EXPECT_FALSE(result);
}

// 4. Inside but has multiple angle difference and velocity conditions
TEST(CheckCloseLaneletConditionTest, AngleDifferenceTooLarge)
{
  auto lanelet = createDummyStraightLanelet(100.0, 3.0);
  constexpr double eps = 1e-6;
  // 1. forward velocity and angle difference is 0
  const TrackedObject obj1_1 = createDummyTrackedObject(50.0, 1.5, 0.0, 10.0);
  const TrackedObject obj1_2 = createDummyTrackedObject(50.0, 1.5, eps, 10.0);
  const TrackedObject obj1_3 = createDummyTrackedObject(50.0, 1.5, -eps, 10.0);
  // 2. forward velocity and angle difference is Inverse
  const TrackedObject obj2_1 = createDummyTrackedObject(50.0, 1.5, M_PI, 10.0);
  const TrackedObject obj2_2 = createDummyTrackedObject(50.0, 1.5, M_PI + eps, 10.0);
  const TrackedObject obj2_3 = createDummyTrackedObject(50.0, 1.5, M_PI - eps, 10.0);
  // 3. backward velocity and angle difference is 0
  const TrackedObject obj3_1 = createDummyTrackedObject(50.0, 1.5, M_PI, -10.0);
  const TrackedObject obj3_2 = createDummyTrackedObject(50.0, 1.5, M_PI + eps, -10.0);
  const TrackedObject obj3_3 = createDummyTrackedObject(50.0, 1.5, M_PI - eps, -10.0);
  // 4. backward velocity and angle difference is Inverse
  const TrackedObject obj4_1 = createDummyTrackedObject(50.0, 1.5, 0.0, -10.0);
  const TrackedObject obj4_2 = createDummyTrackedObject(50.0, 1.5, eps, -10.0);
  const TrackedObject obj4_3 = createDummyTrackedObject(50.0, 1.5, -eps, -10.0);

  // 1 and 3 should be true, 2 and 4 should be false
  EXPECT_TRUE(checkCloseLaneletCondition({0.0, lanelet}, obj1_1, 5.0, M_PI / 4));
  EXPECT_TRUE(checkCloseLaneletCondition({0.0, lanelet}, obj1_2, 5.0, M_PI / 4));
  EXPECT_TRUE(checkCloseLaneletCondition({0.0, lanelet}, obj1_3, 5.0, M_PI / 4));
  EXPECT_TRUE(checkCloseLaneletCondition({0.0, lanelet}, obj3_1, 5.0, M_PI / 4));
  EXPECT_TRUE(checkCloseLaneletCondition({0.0, lanelet}, obj3_2, 5.0, M_PI / 4));
  EXPECT_TRUE(checkCloseLaneletCondition({0.0, lanelet}, obj3_3, 5.0, M_PI / 4));

  EXPECT_FALSE(checkCloseLaneletCondition({0.0, lanelet}, obj2_1, 5.0, M_PI / 4));
  EXPECT_FALSE(checkCloseLaneletCondition({0.0, lanelet}, obj2_2, 5.0, M_PI / 4));
  EXPECT_FALSE(checkCloseLaneletCondition({0.0, lanelet}, obj2_3, 5.0, M_PI / 4));
  EXPECT_FALSE(checkCloseLaneletCondition({0.0, lanelet}, obj4_1, 5.0, M_PI / 4));
  EXPECT_FALSE(checkCloseLaneletCondition({0.0, lanelet}, obj4_2, 5.0, M_PI / 4));
  EXPECT_FALSE(checkCloseLaneletCondition({0.0, lanelet}, obj4_3, 5.0, M_PI / 4));
}
