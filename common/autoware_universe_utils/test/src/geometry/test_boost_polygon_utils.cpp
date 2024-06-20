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

#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>

using autoware::universe_utils::Polygon2d;

namespace
{
geometry_msgs::msg::Point32 createPoint32(const double x, const double y)
{
  geometry_msgs::msg::Point32 p;
  p.x = x;
  p.y = y;
  p.z = 0.0;

  return p;
}

geometry_msgs::msg::Pose createPose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = 0.0;
  p.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);

  return p;
}
}  // namespace

TEST(boost_geometry, boost_isClockwise)
{
  using autoware::universe_utils::isClockwise;

  // empty
  Polygon2d empty_polygon;
  EXPECT_THROW(isClockwise(empty_polygon), std::out_of_range);

  // normal case
  Polygon2d clock_wise_polygon{{{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 0.0}}};
  EXPECT_TRUE(isClockwise(clock_wise_polygon));

  Polygon2d anti_clock_wise_polygon{{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}}};
  EXPECT_FALSE(isClockwise(anti_clock_wise_polygon));

  // duplicated
  Polygon2d duplicated_clock_wise_polygon{
    {{0.0, 0.0}, {0.0, 1.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 0.0}}};
  EXPECT_TRUE(isClockwise(duplicated_clock_wise_polygon));

  Polygon2d duplicated_anti_clock_wise_polygon{
    {{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}}};
  EXPECT_FALSE(isClockwise(duplicated_anti_clock_wise_polygon));
}

TEST(boost_geometry, boost_inverseClockwise)
{
  using autoware::universe_utils::inverseClockwise;
  using autoware::universe_utils::isClockwise;

  // empty
  Polygon2d empty_polygon;
  const auto reversed_empty_polygon = inverseClockwise(empty_polygon);
  EXPECT_EQ(static_cast<int>(reversed_empty_polygon.outer().size()), 0);

  // normal case
  Polygon2d clock_wise_polygon{{{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 0.0}}};
  EXPECT_FALSE(isClockwise(inverseClockwise(clock_wise_polygon)));

  Polygon2d anti_clock_wise_polygon{{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}}};
  EXPECT_TRUE(isClockwise(inverseClockwise(anti_clock_wise_polygon)));

  // duplicated
  Polygon2d duplicated_clock_wise_polygon{
    {{0.0, 0.0}, {0.0, 1.0}, {0.0, 1.0}, {1.0, 1.0}, {1.0, 0.0}, {0.0, 0.0}}};
  EXPECT_FALSE(isClockwise(inverseClockwise(duplicated_clock_wise_polygon)));

  Polygon2d duplicated_anti_clock_wise_polygon{
    {{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.0}}};
  EXPECT_TRUE(isClockwise(inverseClockwise(duplicated_anti_clock_wise_polygon)));
}

TEST(boost_geometry, boost_rotatePolygon)
{
  constexpr double epsilon = 1e-6;
  using autoware::universe_utils::rotatePolygon;

  // empty
  geometry_msgs::msg::Polygon empty_polygon;
  const auto rotated_empty_polygon = rotatePolygon(empty_polygon, 0.0);
  EXPECT_EQ(static_cast<int>(rotated_empty_polygon.points.size()), 0);

  // normal case
  geometry_msgs::msg::Polygon clock_wise_polygon;
  clock_wise_polygon.points.push_back(createPoint32(0.0, 0.0));
  clock_wise_polygon.points.push_back(createPoint32(0.0, 1.0));
  clock_wise_polygon.points.push_back(createPoint32(1.0, 1.0));
  clock_wise_polygon.points.push_back(createPoint32(1.0, 0.0));
  clock_wise_polygon.points.push_back(createPoint32(0.0, 0.0));
  const auto rotated_clock_wise_polygon = rotatePolygon(clock_wise_polygon, M_PI_4);

  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(0).x, 0.0);
  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(0).y, 0.0);
  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(1).x, -0.70710676908493042);
  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(1).y, 0.70710676908493042);
  EXPECT_NEAR(rotated_clock_wise_polygon.points.at(2).x, 0.0, epsilon);
  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(2).y, 1.4142135381698608);
  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(3).x, 0.70710676908493042);
  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(3).y, 0.70710676908493042);
  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(4).x, 0.0);
  EXPECT_DOUBLE_EQ(rotated_clock_wise_polygon.points.at(4).y, 0.0);
}

TEST(boost_geometry, boost_toPolygon2d)
{
  using autoware::universe_utils::toPolygon2d;

  {  // bounding box
    const double x = 1.0;
    const double y = 1.0;

    const auto pose = createPose(1.0, 1.0, M_PI_4);
    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    shape.dimensions.x = x;
    shape.dimensions.y = y;

    const auto poly = toPolygon2d(pose, shape);
    EXPECT_DOUBLE_EQ(poly.outer().at(0).x(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(0).y(), 1.7071067811865475);
    EXPECT_DOUBLE_EQ(poly.outer().at(1).x(), 1.7071067811865475);
    EXPECT_DOUBLE_EQ(poly.outer().at(1).y(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(2).x(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(2).y(), 0.29289321881345254);
    EXPECT_DOUBLE_EQ(poly.outer().at(3).x(), 0.29289321881345254);
    EXPECT_DOUBLE_EQ(poly.outer().at(3).y(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(4).x(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(4).y(), 1.7071067811865475);
  }

  {  // cylinder
    const double diameter = 1.0;

    const auto pose = createPose(1.0, 1.0, M_PI_4);
    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::CYLINDER;
    shape.dimensions.x = diameter;

    const auto poly = toPolygon2d(pose, shape);
    EXPECT_DOUBLE_EQ(poly.outer().at(0).x(), 1.4330127018922194);
    EXPECT_DOUBLE_EQ(poly.outer().at(0).y(), 1.25);
    EXPECT_DOUBLE_EQ(poly.outer().at(1).x(), 1.4330127018922194);
    EXPECT_DOUBLE_EQ(poly.outer().at(1).y(), 0.75);
    EXPECT_DOUBLE_EQ(poly.outer().at(2).x(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(2).y(), 0.5);
    EXPECT_DOUBLE_EQ(poly.outer().at(3).x(), 0.56698729810778059);
    EXPECT_DOUBLE_EQ(poly.outer().at(3).y(), 0.75);
    EXPECT_DOUBLE_EQ(poly.outer().at(4).x(), 0.56698729810778081);
    EXPECT_DOUBLE_EQ(poly.outer().at(4).y(), 1.25);
    EXPECT_DOUBLE_EQ(poly.outer().at(5).x(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(5).y(), 1.5);
  }

  {  // polygon
    const double x = 0.5;
    const double y = 0.5;

    const auto pose = createPose(1.0, 1.0, M_PI_4);
    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::POLYGON;
    shape.footprint.points.push_back(createPoint32(-x, -y));
    shape.footprint.points.push_back(createPoint32(-x, y));
    shape.footprint.points.push_back(createPoint32(x, y));
    shape.footprint.points.push_back(createPoint32(x, -y));

    const auto poly = toPolygon2d(pose, shape);
    EXPECT_DOUBLE_EQ(poly.outer().at(0).x(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(0).y(), 0.29289323091506958);
    EXPECT_DOUBLE_EQ(poly.outer().at(1).x(), 0.29289323091506958);
    EXPECT_DOUBLE_EQ(poly.outer().at(1).y(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(2).x(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(2).y(), 1.7071067690849304);
    EXPECT_DOUBLE_EQ(poly.outer().at(3).x(), 1.7071067690849304);
    EXPECT_DOUBLE_EQ(poly.outer().at(3).y(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(4).x(), 1.0);
    EXPECT_DOUBLE_EQ(poly.outer().at(4).y(), 0.29289323091506958);
  }
}

TEST(boost_geometry, boost_toFootprint)
{
  using autoware::universe_utils::toFootprint;

  // from base link
  {
    const double x = 1.0;
    const double y = 1.0;

    const auto base_link_pose = createPose(1.0, 1.0, M_PI_4);
    const double base_to_front = 4.0;
    const double base_to_back = 1.0;
    const double width = 2.0;

    const auto poly = toFootprint(base_link_pose, base_to_front, base_to_back, width);
    EXPECT_DOUBLE_EQ(poly.outer().at(0).x(), 3.0 / std::sqrt(2) + x);
    EXPECT_DOUBLE_EQ(poly.outer().at(0).y(), 5.0 / std::sqrt(2) + y);
    EXPECT_DOUBLE_EQ(poly.outer().at(1).x(), 5.0 / std::sqrt(2) + x);
    EXPECT_DOUBLE_EQ(poly.outer().at(1).y(), 3.0 / std::sqrt(2) + y);
    EXPECT_DOUBLE_EQ(poly.outer().at(2).x(), x);
    EXPECT_DOUBLE_EQ(poly.outer().at(2).y(), y - std::sqrt(2));
    EXPECT_DOUBLE_EQ(poly.outer().at(3).x(), x - std::sqrt(2));
    EXPECT_DOUBLE_EQ(poly.outer().at(3).y(), y);
    EXPECT_DOUBLE_EQ(poly.outer().at(4).x(), 3.0 / std::sqrt(2) + x);
    EXPECT_DOUBLE_EQ(poly.outer().at(4).y(), 5.0 / std::sqrt(2) + y);
  }
}

TEST(boost_geometry, boost_getArea)
{
  using autoware::universe_utils::getArea;

  {  // bounding box
    const double x = 1.0;
    const double y = 2.0;

    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    shape.dimensions.x = x;
    shape.dimensions.y = y;

    const double area = getArea(shape);
    EXPECT_DOUBLE_EQ(area, x * y);
  }

  {  // cylinder
    const double diameter = 1.0;

    autoware_perception_msgs::msg::Shape shape;
    shape.type = autoware_perception_msgs::msg::Shape::CYLINDER;
    shape.dimensions.x = diameter;

    const double area = getArea(shape);
    EXPECT_DOUBLE_EQ(area, std::pow(diameter / 2.0, 2.0) * M_PI);
  }

  {  // polygon
    const double x = 1.0;
    const double y = 2.0;

    // clock wise
    autoware_perception_msgs::msg::Shape clock_wise_shape;
    clock_wise_shape.type = autoware_perception_msgs::msg::Shape::POLYGON;
    clock_wise_shape.footprint.points.push_back(createPoint32(0.0, 0.0));
    clock_wise_shape.footprint.points.push_back(createPoint32(0.0, y));
    clock_wise_shape.footprint.points.push_back(createPoint32(x, y));
    clock_wise_shape.footprint.points.push_back(createPoint32(x, 0.0));

    const double clock_wise_area = getArea(clock_wise_shape);
    EXPECT_DOUBLE_EQ(clock_wise_area, -x * y);

    // anti clock wise
    autoware_perception_msgs::msg::Shape anti_clock_wise_shape;
    anti_clock_wise_shape.type = autoware_perception_msgs::msg::Shape::POLYGON;
    anti_clock_wise_shape.footprint.points.push_back(createPoint32(0.0, 0.0));
    anti_clock_wise_shape.footprint.points.push_back(createPoint32(x, 0.0));
    anti_clock_wise_shape.footprint.points.push_back(createPoint32(x, y));
    anti_clock_wise_shape.footprint.points.push_back(createPoint32(0.0, y));

    const double anti_clock_wise_area = getArea(anti_clock_wise_shape);
    EXPECT_DOUBLE_EQ(anti_clock_wise_area, x * y);
  }
}

TEST(boost_geometry, boost_expandPolygon)
{
  using autoware::universe_utils::expandPolygon;

  {  // box with a certain offset
    Polygon2d box_poly{{{-1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}}};
    const auto expanded_poly = expandPolygon(box_poly, 1.0);

    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(0).x(), -2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(0).y(), -2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(1).x(), -2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(1).y(), 2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(2).x(), 2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(2).y(), 2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(3).x(), 2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(3).y(), -2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(4).x(), -2.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(4).y(), -2.0);
  }

  {  // box with no offset
    Polygon2d box_poly{{{-1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}}};
    const auto expanded_poly = expandPolygon(box_poly, 0.0);

    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(0).x(), -1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(0).y(), -1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(1).x(), -1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(1).y(), 1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(2).x(), 1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(2).y(), 1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(3).x(), 1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(3).y(), -1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(4).x(), -1.0);
    EXPECT_DOUBLE_EQ(expanded_poly.outer().at(4).y(), -1.0);
  }

  {  // empty polygon
    Polygon2d empty_poly;
    EXPECT_THROW(expandPolygon(empty_poly, 1.0), std::out_of_range);
  }
}
