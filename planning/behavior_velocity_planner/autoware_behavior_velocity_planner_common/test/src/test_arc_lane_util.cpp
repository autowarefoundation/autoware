// Copyright 2021 Tier IV, Inc.
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

#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>

#include <gtest/gtest.h>

#include <utility>

using PathIndexWithPoint2d =
  autoware::behavior_velocity_planner::arc_lane_utils::PathIndexWithPoint2d;
using LineString2d = autoware::behavior_velocity_planner::LineString2d;
using Point2d = autoware::behavior_velocity_planner::Point2d;
namespace arc_lane_utils = autoware::behavior_velocity_planner::arc_lane_utils;

namespace
{
geometry_msgs::msg::Point createPoint(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}
}  // namespace

TEST(findCollisionSegment, nominal)
{
  /**
   * find forward collision segment by stop line
   *                s
   *  | = | = | = | s | = |
   *  0   1   2   3 s  4   5
   *
   **/
  auto path = test::generatePath(0.0, 0.0, 5.0, 0.0, 6);

  LineString2d stop_line;
  stop_line.emplace_back(Point2d(3.5, 3.0));
  stop_line.emplace_back(Point2d(3.5, -3.0));
  auto segment = arc_lane_utils::findCollisionSegment(path, stop_line);
  EXPECT_EQ(segment->first, static_cast<size_t>(3));
  EXPECT_DOUBLE_EQ(segment->second.x, 3.5);
  EXPECT_DOUBLE_EQ(segment->second.y, 0.0);
}

TEST(findOffsetSegment, case_forward_offset_segment)
{
  auto path = test::generatePath(0.0, 0.0, 5.0, 0.0, 6);
  /**
   * find offset segment forward
   *  | = | = | = | c | o |
   *  0   1   2   3   4   5
   **/
  const int collision_segment_idx = 3;
  const auto collision_point = createPoint(3.5, 0.0, 0.0);
  const auto & collision_segment = std::make_pair(collision_segment_idx, collision_point);
  // nominal
  {
    double offset_length = 1.0;
    const auto offset_segment =
      arc_lane_utils::findOffsetSegment(path, collision_segment, offset_length);
    const auto front_idx = offset_segment->first;
    EXPECT_EQ(front_idx, static_cast<size_t>(4));
    EXPECT_DOUBLE_EQ(offset_segment->second, 0.5);
  }
  // boundary condition
  {
    double offset_length = INFINITY;
    const auto offset_segment =
      arc_lane_utils::findOffsetSegment(path, collision_segment, offset_length);
    EXPECT_FALSE(offset_segment);
  }
}

TEST(findOffsetSegment, case_backward_offset_segment)
{
  auto path = test::generatePath(0.0, 0.0, 5.0, 0.0, 6);
  /**
   * find offset segment forward
   *  | = | = | o | c | = |
   *  0   1   2   3   4   5
   **/
  const int collision_segment_idx = 3;
  const auto collision_point = createPoint(3.5, 0.0, 0.0);
  const auto & collision_segment = std::make_pair(collision_segment_idx, collision_point);
  // nominal
  {
    double offset_length = -1.0;
    const auto offset_segment =
      arc_lane_utils::findOffsetSegment(path, collision_segment, offset_length);
    const auto front_idx = offset_segment->first;
    EXPECT_EQ(front_idx, static_cast<size_t>(2));
    EXPECT_DOUBLE_EQ(offset_segment->second, 0.5);
  }
  // boundary condition
  {
    double offset_length = -INFINITY;
    const auto offset_segment =
      arc_lane_utils::findOffsetSegment(path, collision_segment, offset_length);
    EXPECT_FALSE(offset_segment);
  }
}

TEST(checkCollision, various_cases)
{
  using autoware::behavior_velocity_planner::arc_lane_utils::checkCollision;
  constexpr double epsilon = 1e-6;

  {  // normal case with collision
    const auto p1 = createPoint(-1.0, 0.0, 0.0);
    const auto p2 = createPoint(2.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, -1.0, 0.0);
    const auto p4 = createPoint(0.0, 2.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_NE(collision, std::nullopt);
    EXPECT_NEAR(collision->x, 0.0, epsilon);
    EXPECT_NEAR(collision->y, 0.0, epsilon);
    EXPECT_NEAR(collision->z, 0.0, epsilon);
  }

  {  // normal case without collision
    const auto p1 = createPoint(1.0, 0.0, 0.0);
    const auto p2 = createPoint(2.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, -1.0, 0.0);
    const auto p4 = createPoint(0.0, 2.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_EQ(collision, std::nullopt);
  }

  {  // normal case without collision
    const auto p1 = createPoint(-1.0, 0.0, 0.0);
    const auto p2 = createPoint(2.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, 1.0, 0.0);
    const auto p4 = createPoint(0.0, 2.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_EQ(collision, std::nullopt);
  }

  {  // line and point
    const auto p1 = createPoint(-1.0, 0.0, 0.0);
    const auto p2 = createPoint(2.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, 0.0, 0.0);
    const auto p4 = createPoint(0.0, 0.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_EQ(collision, std::nullopt);
  }

  {  // point and line
    const auto p1 = createPoint(0.0, 0.0, 0.0);
    const auto p2 = createPoint(0.0, 0.0, 0.0);
    const auto p3 = createPoint(-1.0, 0.0, 0.0);
    const auto p4 = createPoint(2.0, 0.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_EQ(collision, std::nullopt);
  }

  {  // collision with edges
    const auto p1 = createPoint(-1.0, 0.0, 0.0);
    const auto p2 = createPoint(0.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, 0.0, 0.0);
    const auto p4 = createPoint(0.0, 2.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_NE(collision, std::nullopt);
    EXPECT_NEAR(collision->x, 0.0, epsilon);
    EXPECT_NEAR(collision->y, 0.0, epsilon);
    EXPECT_NEAR(collision->z, 0.0, epsilon);
  }

  {  // collision with edge
    const auto p1 = createPoint(-1.0, 0.0, 0.0);
    const auto p2 = createPoint(1.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, 0.0, 0.0);
    const auto p4 = createPoint(0.0, 1.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_NE(collision, std::nullopt);
    EXPECT_NEAR(collision->x, 0.0, epsilon);
    EXPECT_NEAR(collision->y, 0.0, epsilon);
    EXPECT_NEAR(collision->z, 0.0, epsilon);
  }

  {  // collision with edge
    const auto p1 = createPoint(-1.0, 0.0, 0.0);
    const auto p2 = createPoint(1.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, -1.0, 0.0);
    const auto p4 = createPoint(0.0, 0.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_NE(collision, std::nullopt);
    EXPECT_NEAR(collision->x, 0.0, epsilon);
    EXPECT_NEAR(collision->y, 0.0, epsilon);
    EXPECT_NEAR(collision->z, 0.0, epsilon);
  }

  {  // collision with edge
    const auto p1 = createPoint(0.0, 0.0, 0.0);
    const auto p2 = createPoint(1.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, -1.0, 0.0);
    const auto p4 = createPoint(0.0, 1.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_NE(collision, std::nullopt);
    EXPECT_NEAR(collision->x, 0.0, epsilon);
    EXPECT_NEAR(collision->y, 0.0, epsilon);
    EXPECT_NEAR(collision->z, 0.0, epsilon);
  }

  {  // collision with edge
    const auto p1 = createPoint(-1.0, 0.0, 0.0);
    const auto p2 = createPoint(0.0, 0.0, 0.0);
    const auto p3 = createPoint(0.0, -1.0, 0.0);
    const auto p4 = createPoint(0.0, 1.0, 0.0);

    const auto collision = checkCollision(p1, p2, p3, p4);
    EXPECT_NE(collision, std::nullopt);
    EXPECT_NEAR(collision->x, 0.0, epsilon);
    EXPECT_NEAR(collision->y, 0.0, epsilon);
    EXPECT_NEAR(collision->z, 0.0, epsilon);
  }
}
