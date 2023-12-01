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

#include "motion_utils/trajectory/path_with_lane_id.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using tier4_autoware_utils::createPoint;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = tier4_autoware_utils::createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

PathWithLaneId generateTestPathWithLaneId(const size_t num_points, const double point_interval)
{
  PathWithLaneId path;
  for (size_t i = 0; i < num_points; ++i) {
    const double x = i * point_interval;
    const double y = 0.0;

    PathPointWithLaneId p;
    p.point.pose = createPose(x, y, 0.0, 0.0, 0.0, 0.0);
    p.lane_ids.push_back(i);
    path.points.push_back(p);
  }

  return path;
}
}  // namespace

TEST(path_with_lane_id, getPathIndexRangeWithLaneId)
{
  using autoware_auto_planning_msgs::msg::PathWithLaneId;
  using motion_utils::getPathIndexRangeWithLaneId;

  // Usual cases
  {
    PathWithLaneId points;
    points.points.resize(6);
    points.points.at(0).lane_ids.push_back(3);
    points.points.at(1).lane_ids.push_back(3);
    points.points.at(2).lane_ids.push_back(1);
    points.points.at(3).lane_ids.push_back(2);
    points.points.at(4).lane_ids.push_back(2);
    points.points.at(5).lane_ids.push_back(2);

    {
      const auto res = getPathIndexRangeWithLaneId(points, 3);
      EXPECT_EQ(res->first, 0U);
      EXPECT_EQ(res->second, 2U);
    }
    {
      const auto res = getPathIndexRangeWithLaneId(points, 1);
      EXPECT_EQ(res->first, 1U);
      EXPECT_EQ(res->second, 3U);
    }
    {
      const auto res = getPathIndexRangeWithLaneId(points, 2);
      EXPECT_EQ(res->first, 2U);
      EXPECT_EQ(res->second, 5U);
    }
    {
      const auto res = getPathIndexRangeWithLaneId(points, 4);
      EXPECT_EQ(res, std::nullopt);
    }
  }

  // Empty points
  {
    PathWithLaneId points;
    const auto res = getPathIndexRangeWithLaneId(points, 4);
    EXPECT_EQ(res, std::nullopt);
  }
}

TEST(path_with_lane_id, findNearestIndexFromLaneId)
{
  using motion_utils::findNearestIndexFromLaneId;
  using motion_utils::findNearestSegmentIndexFromLaneId;

  const auto path = generateTestPathWithLaneId(10, 1.0);

  // Normal cases
  {
    auto modified_path = path;
    for (size_t i = 0; i < 10; ++i) {
      modified_path.points.at(i).lane_ids = {100};
    }
    EXPECT_EQ(findNearestIndexFromLaneId(modified_path, createPoint(2.4, 1.3, 0.0), 100), 2U);
    EXPECT_EQ(
      findNearestSegmentIndexFromLaneId(modified_path, createPoint(2.4, 1.3, 0.0), 100), 2U);
  }

  {
    auto modified_path = path;
    for (size_t i = 3; i < 6; ++i) {
      modified_path.points.at(i).lane_ids = {100};
    }
    EXPECT_EQ(findNearestIndexFromLaneId(modified_path, createPoint(4.1, 0.3, 0.0), 100), 4U);
    EXPECT_EQ(
      findNearestSegmentIndexFromLaneId(modified_path, createPoint(4.1, 0.3, 0.0), 100), 4U);
  }

  {
    auto modified_path = path;
    for (size_t i = 8; i < 9; ++i) {
      modified_path.points.at(i).lane_ids = {100};
    }
    EXPECT_EQ(findNearestIndexFromLaneId(modified_path, createPoint(8.5, -0.5, 0.0), 100), 8U);
    EXPECT_EQ(
      findNearestSegmentIndexFromLaneId(modified_path, createPoint(8.5, -0.5, 0.0), 100), 8U);
  }

  // Nearest is not within range
  {
    auto modified_path = path;
    for (size_t i = 3; i < 9; ++i) {
      modified_path.points.at(i).lane_ids = {100};
    }
    EXPECT_EQ(findNearestIndexFromLaneId(modified_path, createPoint(2.4, 1.3, 0.0), 100), 2U);
    EXPECT_EQ(
      findNearestSegmentIndexFromLaneId(modified_path, createPoint(2.4, 1.3, 0.0), 100), 2U);
  }

  // Path does not contain lane_id.
  {
    EXPECT_EQ(findNearestIndexFromLaneId(path, createPoint(2.4, 1.3, 0.0), 100), 2U);
    EXPECT_EQ(findNearestSegmentIndexFromLaneId(path, createPoint(2.4, 1.3, 0.0), 100), 2U);
  }

  // Empty points
  EXPECT_THROW(
    findNearestIndexFromLaneId(PathWithLaneId{}, createPoint(2.4, 1.3, 0.0), 100),
    std::invalid_argument);
  EXPECT_THROW(
    findNearestSegmentIndexFromLaneId(PathWithLaneId{}, createPoint(2.4, 1.3, 0.0), 100),
    std::invalid_argument);
}

// NOTE: This test is temporary for the current implementation.
TEST(path_with_lane_id, convertToRearWheelCenter)
{
  using motion_utils::convertToRearWheelCenter;

  PathWithLaneId path;

  PathPointWithLaneId p1;
  p1.point.pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  path.points.push_back(p1);
  PathPointWithLaneId p2;
  p2.point.pose = createPose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0);
  path.points.push_back(p2);
  PathPointWithLaneId p3;
  p3.point.pose = createPose(2.0, 4.0, 0.0, 0.0, 0.0, 0.0);
  path.points.push_back(p3);

  const auto cog_path = convertToRearWheelCenter(path, 1.0);

  constexpr double epsilon = 1e-6;
  EXPECT_NEAR(cog_path.points.at(0).point.pose.position.x, -0.4472136, epsilon);
  EXPECT_NEAR(cog_path.points.at(0).point.pose.position.y, -0.8944272, epsilon);
  EXPECT_NEAR(cog_path.points.at(1).point.pose.position.x, 0.5527864, epsilon);
  EXPECT_NEAR(cog_path.points.at(1).point.pose.position.y, 1.1055728, epsilon);
  EXPECT_NEAR(cog_path.points.at(2).point.pose.position.x, 2.0, epsilon);
  EXPECT_NEAR(cog_path.points.at(2).point.pose.position.y, 4.0, epsilon);
}
