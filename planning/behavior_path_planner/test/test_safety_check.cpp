// Copyright 2023 TIER IV, Inc.
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

#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

constexpr double epsilon = 1e-6;

using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebug;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

namespace bg = boost::geometry;

namespace
{
void appendPointToPolygon(Polygon2d & polygon, const Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

void appendPointToPolygon(Polygon2d & polygon, const Point2d & point)
{
  bg::append(polygon.outer(), point);
}

Polygon2d createPolygon(const std::vector<Point> & points)
{
  using tier4_autoware_utils::inverseClockwise;
  using tier4_autoware_utils::isClockwise;

  Polygon2d polygon;
  for (const auto & point : points) {
    appendPointToPolygon(polygon, point);
  }

  if (!polygon.outer().empty()) {
    appendPointToPolygon(polygon, polygon.outer().front());
  }
  return isClockwise(polygon) ? polygon : inverseClockwise(polygon);
}
}  // namespace

TEST(BehaviorPathPlanningLaneChangeUtilsTest, calcDistanceForRectangle)
{
  using behavior_path_planner::utils::safety_check::calcLateralDistance;
  using behavior_path_planner::utils::safety_check::calcLongitudinalDistance;
  using tier4_autoware_utils::createPoint;

  // same position for lateral position
  {
    std::vector<Point> front_points;
    front_points.emplace_back(createPoint(0.0, 0.0, 0.0));
    front_points.emplace_back(createPoint(5.0, 0.0, 0.0));
    front_points.emplace_back(createPoint(5.0, 2.0, 0.0));
    front_points.emplace_back(createPoint(0.0, 2.0, 0.0));
    const auto front_polygon = createPolygon(front_points);

    std::vector<Point> rear_points;
    rear_points.emplace_back(createPoint(0.0, 4.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 4.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 6.0, 0.0));
    rear_points.emplace_back(createPoint(0.0, 6.0, 0.0));
    const auto rear_polygon = createPolygon(rear_points);

    // lateral distance
    EXPECT_NEAR(calcLateralDistance(front_polygon, rear_polygon), 2.0, epsilon);
    EXPECT_NEAR(calcLateralDistance(rear_polygon, front_polygon), 2.0, epsilon);

    // longitudinal distance
    EXPECT_NEAR(calcLongitudinalDistance(front_polygon, rear_polygon), 0.0, epsilon);
    EXPECT_NEAR(calcLongitudinalDistance(rear_polygon, front_polygon), 0.0, epsilon);
  }

  {
    std::vector<Point> rear_points;
    rear_points.emplace_back(createPoint(0.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 2.0, 0.0));
    rear_points.emplace_back(createPoint(0.0, 2.0, 0.0));
    const auto rear_polygon = createPolygon(rear_points);

    std::vector<Point> front_points;
    front_points.emplace_back(createPoint(10.0, 4.0, 0.0));
    front_points.emplace_back(createPoint(15.0, 4.0, 0.0));
    front_points.emplace_back(createPoint(15.0, 6.0, 0.0));
    front_points.emplace_back(createPoint(10.0, 6.0, 0.0));
    const auto front_polygon = createPolygon(front_points);

    // lateral distance
    EXPECT_NEAR(calcLateralDistance(front_polygon, rear_polygon), 2.0, epsilon);

    // longitudinal distance
    EXPECT_NEAR(calcLongitudinalDistance(front_polygon, rear_polygon), 5.0, epsilon);
  }

  {
    std::vector<Point> rear_points;
    rear_points.emplace_back(createPoint(0.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 2.0, 0.0));
    rear_points.emplace_back(createPoint(0.0, 2.0, 0.0));
    const auto rear_polygon = createPolygon(rear_points);

    std::vector<Point> front_points;
    front_points.emplace_back(createPoint(4.0, 4.0, 0.0));
    front_points.emplace_back(createPoint(9.0, 4.0, 0.0));
    front_points.emplace_back(createPoint(9.0, 6.0, 0.0));
    front_points.emplace_back(createPoint(4.0, 6.0, 0.0));
    const auto front_polygon = createPolygon(front_points);

    // lateral distance
    EXPECT_NEAR(calcLateralDistance(front_polygon, rear_polygon), 2.0, epsilon);

    // longitudinal distance
    EXPECT_NEAR(calcLongitudinalDistance(front_polygon, rear_polygon), 0.0, epsilon);
  }

  {
    std::vector<Point> rear_points;
    rear_points.emplace_back(createPoint(0.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 2.0, 0.0));
    rear_points.emplace_back(createPoint(0.0, 2.0, 0.0));
    const auto rear_polygon = createPolygon(rear_points);

    std::vector<Point> front_points;
    front_points.emplace_back(createPoint(10.0, 0.0, 0.0));
    front_points.emplace_back(createPoint(15.0, 0.0, 0.0));
    front_points.emplace_back(createPoint(15.0, 2.0, 0.0));
    front_points.emplace_back(createPoint(10.0, 2.0, 0.0));
    const auto front_polygon = createPolygon(front_points);

    // lateral distance
    EXPECT_NEAR(calcLateralDistance(front_polygon, rear_polygon), 0.0, epsilon);

    // longitudinal distance
    EXPECT_NEAR(calcLongitudinalDistance(front_polygon, rear_polygon), 5.0, epsilon);
  }

  {
    std::vector<Point> rear_points;
    rear_points.emplace_back(createPoint(0.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0, 2.0, 0.0));
    rear_points.emplace_back(createPoint(0.0, 2.0, 0.0));
    const auto rear_polygon = createPolygon(rear_points);

    std::vector<Point> front_points;
    front_points.emplace_back(createPoint(5.0, 4.0, 0.0));
    front_points.emplace_back(createPoint(10.0, 4.0, 0.0));
    front_points.emplace_back(createPoint(10.0, 6.0, 0.0));
    front_points.emplace_back(createPoint(5.0, 6.0, 0.0));
    const auto front_polygon = createPolygon(front_points);

    // lateral distance
    EXPECT_NEAR(calcLateralDistance(front_polygon, rear_polygon), 2.0, epsilon);

    // longitudinal distance
    EXPECT_NEAR(calcLongitudinalDistance(front_polygon, rear_polygon), 0.0, epsilon);
  }

  // rotate rear to 30 degrees
  {
    std::vector<Point> rear_points;
    rear_points.emplace_back(createPoint(0.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0 * std::sqrt(3) / 2.0, 2.5, 0.0));
    rear_points.emplace_back(createPoint(5.0 * std::sqrt(3) / 2.0 - 1.0, 2.5 + std::sqrt(3), 0.0));
    rear_points.emplace_back(createPoint(-1.0, std::sqrt(3), 0.0));
    const auto rear_polygon = createPolygon(rear_points);

    std::vector<Point> front_points;
    front_points.emplace_back(createPoint(5.0, 4.0, 0.0));
    front_points.emplace_back(createPoint(10.0, 4.0, 0.0));
    front_points.emplace_back(createPoint(10.0, 6.0, 0.0));
    front_points.emplace_back(createPoint(5.0, 6.0, 0.0));
    const auto front_polygon = createPolygon(front_points);

    // lateral distance
    EXPECT_NEAR(calcLateralDistance(front_polygon, rear_polygon), 0.0, epsilon);

    // longitudinal distance
    EXPECT_NEAR(
      calcLongitudinalDistance(front_polygon, rear_polygon), 5.0 - 5 * std::sqrt(3) / 2.0, epsilon);
  }

  // rotate rear to 30 degrees
  {
    std::vector<Point> rear_points;
    rear_points.emplace_back(createPoint(0.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0 * std::sqrt(3) / 2.0, 2.5, 0.0));
    rear_points.emplace_back(createPoint(5.0 * std::sqrt(3) / 2.0 - 1.0, 2.5 + std::sqrt(3), 0.0));
    rear_points.emplace_back(createPoint(-1.0, std::sqrt(3), 0.0));
    const auto rear_polygon = createPolygon(rear_points);

    std::vector<Point> front_points;
    front_points.emplace_back(createPoint(5.0, 5.0, 0.0));
    front_points.emplace_back(createPoint(10.0, 5.0, 0.0));
    front_points.emplace_back(createPoint(10.0, 7.0, 0.0));
    front_points.emplace_back(createPoint(5.0, 7.0, 0.0));
    const auto front_polygon = createPolygon(front_points);

    // lateral distance
    EXPECT_NEAR(
      calcLateralDistance(front_polygon, rear_polygon), 5.0 - 2.5 - std::sqrt(3), epsilon);

    // longitudinal distance
    EXPECT_NEAR(
      calcLongitudinalDistance(front_polygon, rear_polygon), 5.0 - 5 * std::sqrt(3) / 2.0, epsilon);
  }

  // rotate both rear and front to 30 degrees
  {
    std::vector<Point> rear_points;
    rear_points.emplace_back(createPoint(0.0, 0.0, 0.0));
    rear_points.emplace_back(createPoint(5.0 * std::sqrt(3) / 2.0, 2.5, 0.0));
    rear_points.emplace_back(createPoint(5.0 * std::sqrt(3) / 2.0 - 1.0, 2.5 + std::sqrt(3), 0.0));
    rear_points.emplace_back(createPoint(-1.0, std::sqrt(3), 0.0));
    const auto rear_polygon = createPolygon(rear_points);

    std::vector<Point> front_points;
    front_points.emplace_back(createPoint(10.0, 5.0, 0.0));
    front_points.emplace_back(createPoint(10 + 5.0 * std::sqrt(3) / 2.0, 5.0 + 2.5, 0.0));
    front_points.emplace_back(
      createPoint(10 + 5.0 * std::sqrt(3) / 2.0 - 1.0, 5.0 + 2.5 + std::sqrt(3), 0.0));
    front_points.emplace_back(createPoint(10 - 1.0, 5.0 + std::sqrt(3), 0.0));
    const auto front_polygon = createPolygon(front_points);

    // lateral distance
    EXPECT_NEAR(
      calcLateralDistance(front_polygon, rear_polygon), 5.0 - 2.5 - std::sqrt(3), epsilon);

    // longitudinal distance
    EXPECT_NEAR(
      calcLongitudinalDistance(front_polygon, rear_polygon), 9.0 - 5 * std::sqrt(3) / 2.0, epsilon);
  }
}
