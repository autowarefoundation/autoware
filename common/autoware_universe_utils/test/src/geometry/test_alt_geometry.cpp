// Copyright 2020-2024 Tier IV, Inc.
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

#include "autoware/universe_utils/geometry/alt_geometry.hpp"
#include "autoware/universe_utils/geometry/random_convex_polygon.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/touches.hpp>
#include <boost/geometry/io/wkt/write.hpp>
#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <string>

constexpr double epsilon = 1e-6;

TEST(alt_geometry, area)
{
  using autoware::universe_utils::area;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Point2d;

  {  // Clockwise
    const Point2d p1 = {0.0, 0.0};
    const Point2d p2 = {0.0, 7.0};
    const Point2d p3 = {4.0, 2.0};
    const Point2d p4 = {2.0, 0.0};
    const auto result = area(ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_NEAR(result, 16.0, epsilon);
  }

  {  // Counter-clockwise
    const Point2d p1 = {0.0, 0.0};
    const Point2d p2 = {2.0, 0.0};
    const Point2d p3 = {4.0, 2.0};
    const Point2d p4 = {0.0, 7.0};
    const auto result = area(ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_NEAR(result, -16.0, epsilon);
  }
}

TEST(alt_geometry, convexHull)
{
  using autoware::universe_utils::convex_hull;
  using autoware::universe_utils::alt::Points2d;

  {
    Points2d points;
    points.push_back({2.0, 1.3});
    points.push_back({2.4, 1.7});
    points.push_back({2.8, 1.8});
    points.push_back({3.4, 1.2});
    points.push_back({3.7, 1.6});
    points.push_back({3.4, 2.0});
    points.push_back({4.1, 3.0});
    points.push_back({5.3, 2.6});
    points.push_back({5.4, 1.2});
    points.push_back({4.9, 0.8});
    points.push_back({2.9, 0.7});
    const auto result = convex_hull(points);

    ASSERT_EQ(result.vertices().size(), 7);
    EXPECT_NEAR(result.vertices().at(0).x(), 2.0, epsilon);
    EXPECT_NEAR(result.vertices().at(0).y(), 1.3, epsilon);
    EXPECT_NEAR(result.vertices().at(1).x(), 2.4, epsilon);
    EXPECT_NEAR(result.vertices().at(1).y(), 1.7, epsilon);
    EXPECT_NEAR(result.vertices().at(2).x(), 4.1, epsilon);
    EXPECT_NEAR(result.vertices().at(2).y(), 3.0, epsilon);
    EXPECT_NEAR(result.vertices().at(3).x(), 5.3, epsilon);
    EXPECT_NEAR(result.vertices().at(3).y(), 2.6, epsilon);
    EXPECT_NEAR(result.vertices().at(4).x(), 5.4, epsilon);
    EXPECT_NEAR(result.vertices().at(4).y(), 1.2, epsilon);
    EXPECT_NEAR(result.vertices().at(5).x(), 4.9, epsilon);
    EXPECT_NEAR(result.vertices().at(5).y(), 0.8, epsilon);
    EXPECT_NEAR(result.vertices().at(6).x(), 2.9, epsilon);
    EXPECT_NEAR(result.vertices().at(6).y(), 0.7, epsilon);
  }
}

TEST(alt_geometry, correct)
{
  using autoware::universe_utils::correct;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Points2d;

  {  // Correctly oriented
    Points2d vertices;
    vertices.push_back({1.0, 1.0});
    vertices.push_back({1.0, -1.0});
    vertices.push_back({-1.0, -1.0});
    vertices.push_back({-1.0, 1.0});
    ConvexPolygon2d poly(vertices);
    correct(poly);

    ASSERT_EQ(poly.vertices().size(), 4);
    EXPECT_NEAR(poly.vertices().at(0).x(), 1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(0).y(), 1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(1).x(), 1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(1).y(), -1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(2).x(), -1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(2).y(), -1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(3).x(), -1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(3).y(), 1.0, epsilon);
  }

  {  // Wrongly oriented
    Points2d vertices;
    vertices.push_back({1.0, 1.0});
    vertices.push_back({-1.0, 1.0});
    vertices.push_back({1.0, -1.0});
    vertices.push_back({-1.0, -1.0});
    ConvexPolygon2d poly(vertices);
    correct(poly);

    ASSERT_EQ(poly.vertices().size(), 4);
    EXPECT_NEAR(poly.vertices().at(0).x(), 1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(0).y(), 1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(1).x(), 1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(1).y(), -1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(2).x(), -1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(2).y(), -1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(3).x(), -1.0, epsilon);
    EXPECT_NEAR(poly.vertices().at(3).y(), 1.0, epsilon);
  }
}

TEST(alt_geometry, coveredBy)
{
  using autoware::universe_utils::covered_by;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Point2d;

  {  // The point is within the polygon
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {1.0, 1.0};
    const Point2d p2 = {1.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const auto result = covered_by(point, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_TRUE(result);
  }

  {  // The point is outside the polygon
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {2.0, 2.0};
    const Point2d p2 = {2.0, 1.0};
    const Point2d p3 = {1.0, 1.0};
    const Point2d p4 = {1.0, 2.0};
    const auto result = covered_by(point, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_FALSE(result);
  }

  {  // The point is on the edge of the polygon
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {2.0, 1.0};
    const Point2d p2 = {2.0, -1.0};
    const Point2d p3 = {0.0, -1.0};
    const Point2d p4 = {0.0, 1.0};
    const auto result = covered_by(point, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_TRUE(result);
  }
}

TEST(alt_geometry, disjoint)
{
  using autoware::universe_utils::disjoint;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Point2d;

  {  // Disjoint
    const Point2d p1 = {0.0, 2.0};
    const Point2d p2 = {-2.0, 0.0};
    const Point2d p3 = {-4.0, 2.0};
    const Point2d p4 = {-2.0, 4.0};
    const Point2d p5 = {2.0, 2.0};
    const Point2d p6 = {4.0, 4.0};
    const Point2d p7 = {6.0, 2.0};
    const Point2d p8 = {4.0, 0.0};
    const auto result =
      disjoint(ConvexPolygon2d({p1, p2, p3, p4}), ConvexPolygon2d({p5, p6, p7, p8}));

    EXPECT_TRUE(result);
  }

  {  // Not disjoint (two polygons share a vertex)
    const Point2d p1 = {0.0, 2.0};
    const Point2d p2 = {-2.0, 0.0};
    const Point2d p3 = {-4.0, 2.0};
    const Point2d p4 = {-2.0, 4.0};
    const Point2d p5 = {0.0, 2.0};
    const Point2d p6 = {2.0, 4.0};
    const Point2d p7 = {4.0, 2.0};
    const Point2d p8 = {2.0, 0.0};
    const auto result =
      disjoint(ConvexPolygon2d({p1, p2, p3, p4}), ConvexPolygon2d({p5, p6, p7, p8}));

    EXPECT_FALSE(result);
  }
}

TEST(alt_geometry, distance)
{
  using autoware::universe_utils::distance;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Point2d;

  {  // Normal setting
    const Point2d p = {0.0, 1.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = distance(p, p1, p2);

    EXPECT_NEAR(result, 1.0, epsilon);
  }

  {
    // The point is out of range of the segment to the start point side
    const Point2d p = {-2.0, 1.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = distance(p, p1, p2);

    EXPECT_NEAR(result, std::sqrt(2), epsilon);
  }

  {
    // The point is out of range of the segment to the end point side
    const Point2d p = {2.0, 1.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = distance(p, p1, p2);

    EXPECT_NEAR(result, std::sqrt(2), epsilon);
  }

  {
    // The point is on the segment
    const Point2d p = {0.0, 0.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = distance(p, p1, p2);

    EXPECT_NEAR(result, 0.0, epsilon);
  }

  {
    // The point is the start point of the segment
    const Point2d p = {-1.0, 0.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = distance(p, p1, p2);

    EXPECT_NEAR(result, 0.0, epsilon);
  }

  {
    // The point is the end point of the segment
    const Point2d p = {1.0, 0.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = distance(p, p1, p2);

    EXPECT_NEAR(result, 0.0, epsilon);
  }

  {  // The segment is a point
    const Point2d p = {0.0, 0.0};
    const Point2d p1 = {1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = distance(p, p1, p2);

    EXPECT_NEAR(result, 1.0, epsilon);
  }

  {  // The point is outside the polygon
    const Point2d p = {0.0, 0.0};
    const Point2d p1 = {3.0, 1.0};
    const Point2d p2 = {3.0, -1.0};
    const Point2d p3 = {1.0, -1.0};
    const Point2d p4 = {1.0, 1.0};
    const auto result = distance(p, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_NEAR(result, 1.0, epsilon);
  }

  {  // The point is within the polygon
    const Point2d p = {0.0, 0.0};
    const Point2d p1 = {2.0, 1.0};
    const Point2d p2 = {2.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const auto result = distance(p, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_NEAR(result, 0.0, epsilon);
  }
}

TEST(alt_geometry, intersects)
{
  using autoware::universe_utils::intersects;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Point2d;

  {  // Normally crossing
    const Point2d p1 = {0.0, -1.0};
    const Point2d p2 = {0.0, 1.0};
    const Point2d p3 = {-1.0, 0.0};
    const Point2d p4 = {1.0, 0.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_TRUE(result);
  }

  {  // No crossing
    const Point2d p1 = {0.0, -1.0};
    const Point2d p2 = {0.0, 1.0};
    const Point2d p3 = {1.0, 0.0};
    const Point2d p4 = {3.0, 0.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_FALSE(result);
  }

  {  // One segment is the point on the other's segment
    const Point2d p1 = {0.0, -1.0};
    const Point2d p2 = {0.0, 1.0};
    const Point2d p3 = {0.0, 0.0};
    const Point2d p4 = {0.0, 0.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_FALSE(result);
  }

  {  // One segment is the point not on the other's segment
    const Point2d p1 = {0.0, -1.0};
    const Point2d p2 = {0.0, 1.0};
    const Point2d p3 = {1.0, 0.0};
    const Point2d p4 = {1.0, 0.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_FALSE(result);
  }

  {  // Both segments are the points which are the same position
    const Point2d p1 = {0.0, 0.0};
    const Point2d p2 = {0.0, 0.0};
    const Point2d p3 = {0.0, 0.0};
    const Point2d p4 = {0.0, 0.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_FALSE(result);
  }

  {  // Both segments are the points which are different position
    const Point2d p1 = {0.0, 1.0};
    const Point2d p2 = {0.0, 1.0};
    const Point2d p3 = {1.0, 0.0};
    const Point2d p4 = {1.0, 0.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_FALSE(result);
  }

  {  // Segments are the same
    const Point2d p1 = {0.0, -1.0};
    const Point2d p2 = {0.0, 1.0};
    const Point2d p3 = {0.0, -1.0};
    const Point2d p4 = {0.0, 1.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_FALSE(result);
  }

  {  // One's edge is on the other's segment
    const Point2d p1 = {0.0, -1.0};
    const Point2d p2 = {0.0, 1.0};
    const Point2d p3 = {0.0, 0.0};
    const Point2d p4 = {1.0, 0.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_TRUE(result);
  }

  {  // One's edge is the same as the other's edge.
    const Point2d p1 = {0.0, -1.0};
    const Point2d p2 = {0.0, 1.0};
    const Point2d p3 = {0.0, -1.0};
    const Point2d p4 = {2.0, -1.0};
    const auto result = intersects(p1, p2, p3, p4);

    EXPECT_TRUE(result);
  }

  {  // One polygon intersects the other
    const Point2d p1 = {1.0, 1.0};
    const Point2d p2 = {1.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const Point2d p5 = {2.0, 2.0};
    const Point2d p6 = {2.0, 0.0};
    const Point2d p7 = {0.0, 0.0};
    const Point2d p8 = {0.0, 2.0};
    const auto result =
      intersects(ConvexPolygon2d({p1, p2, p3, p4}), ConvexPolygon2d({p5, p6, p7, p8}));

    EXPECT_TRUE(result);
  }

  {  // Two polygons do not intersect each other
    const Point2d p1 = {1.0, 1.0};
    const Point2d p2 = {1.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const Point2d p5 = {3.0, 3.0};
    const Point2d p6 = {3.0, 2.0};
    const Point2d p7 = {2.0, 2.0};
    const Point2d p8 = {2.0, 3.0};
    const auto result =
      intersects(ConvexPolygon2d({p1, p2, p3, p4}), ConvexPolygon2d({p5, p6, p7, p8}));

    EXPECT_FALSE(result);
  }

  {  // Two polygons share a vertex
    const Point2d p1 = {1.0, 1.0};
    const Point2d p2 = {1.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const Point2d p5 = {2.0, 2.0};
    const Point2d p6 = {2.0, 1.0};
    const Point2d p7 = {1.0, 1.0};
    const Point2d p8 = {1.0, 2.0};
    const auto result =
      intersects(ConvexPolygon2d({p1, p2, p3, p4}), ConvexPolygon2d({p5, p6, p7, p8}));

    EXPECT_FALSE(result);
  }
}

TEST(alt_geometry, isAbove)
{
  using autoware::universe_utils::is_above;
  using autoware::universe_utils::alt::Point2d;

  {  // The point is above the line
    const Point2d point = {0.0, 1.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = is_above(point, p1, p2);

    EXPECT_TRUE(result);
  }

  {  // The point is below the line
    const Point2d point = {0.0, -1.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = is_above(point, p1, p2);

    EXPECT_FALSE(result);
  }

  {  // The point is on the line
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = is_above(point, p1, p2);

    EXPECT_FALSE(result);
  }
}

TEST(alt_geometry, isClockwise)
{
  using autoware::universe_utils::is_clockwise;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Point2d;

  {  // Clockwise
    const Point2d p1 = {0.0, 0.0};
    const Point2d p2 = {0.0, 7.0};
    const Point2d p3 = {4.0, 2.0};
    const Point2d p4 = {2.0, 0.0};
    const auto result = is_clockwise(ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_TRUE(result);
  }

  {  // Counter-clockwise
    const Point2d p1 = {0.0, 0.0};
    const Point2d p2 = {2.0, 0.0};
    const Point2d p3 = {4.0, 2.0};
    const Point2d p4 = {0.0, 7.0};
    const auto result = is_clockwise(ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_FALSE(result);
  }
}

TEST(alt_geometry, touches)
{
  using autoware::universe_utils::touches;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Point2d;

  {
    // The point is on the segment
    const Point2d p = {0.0, 0.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = touches(p, p1, p2);

    EXPECT_TRUE(result);
  }

  {
    // The point is the start point of the segment
    const Point2d p = {-1.0, 0.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = touches(p, p1, p2);

    EXPECT_TRUE(result);
  }

  {
    // The point is the end point of the segment
    const Point2d p = {1.0, 0.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = touches(p, p1, p2);

    EXPECT_TRUE(result);
  }

  {  // The point is not on the segment
    const Point2d p = {0.0, 1.0};
    const Point2d p1 = {-1.0, 0.0};
    const Point2d p2 = {1.0, 0.0};
    const auto result = touches(p, p1, p2);

    EXPECT_FALSE(result);
  }

  {  // The point is on the edge of the polygon
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {2.0, 1.0};
    const Point2d p2 = {2.0, -1.0};
    const Point2d p3 = {0.0, -1.0};
    const Point2d p4 = {0.0, 1.0};
    const auto result = touches(point, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_TRUE(result);
  }

  {  // The point is outside the polygon
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {2.0, 2.0};
    const Point2d p2 = {2.0, 1.0};
    const Point2d p3 = {1.0, 1.0};
    const Point2d p4 = {1.0, 2.0};
    const auto result = touches(point, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_FALSE(result);
  }
}

TEST(alt_geometry, within)
{
  using autoware::universe_utils::within;
  using autoware::universe_utils::alt::ConvexPolygon2d;
  using autoware::universe_utils::alt::Point2d;

  {  // The point is within the polygon
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {1.0, 1.0};
    const Point2d p2 = {1.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const auto result = within(point, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_TRUE(result);
  }

  {  // The point is outside the polygon
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {2.0, 2.0};
    const Point2d p2 = {2.0, 1.0};
    const Point2d p3 = {1.0, 1.0};
    const Point2d p4 = {1.0, 2.0};
    const auto result = within(point, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_FALSE(result);
  }

  {  // The point is on the edge of the polygon
    const Point2d point = {0.0, 0.0};
    const Point2d p1 = {2.0, 1.0};
    const Point2d p2 = {2.0, -1.0};
    const Point2d p3 = {0.0, -1.0};
    const Point2d p4 = {0.0, 1.0};
    const auto result = within(point, ConvexPolygon2d({p1, p2, p3, p4}));

    EXPECT_FALSE(result);
  }

  {  // One polygon is within the other
    const Point2d p1 = {1.0, 1.0};
    const Point2d p2 = {1.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const Point2d p5 = {2.0, 2.0};
    const Point2d p6 = {2.0, -2.0};
    const Point2d p7 = {-2.0, -2.0};
    const Point2d p8 = {-2.0, 2.0};
    const auto result =
      within(ConvexPolygon2d({p1, p2, p3, p4}), ConvexPolygon2d({p5, p6, p7, p8}));

    EXPECT_TRUE(result);
  }

  {  // One polygon is outside the other
    const Point2d p1 = {1.0, 1.0};
    const Point2d p2 = {1.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const Point2d p5 = {3.0, 3.0};
    const Point2d p6 = {3.0, 2.0};
    const Point2d p7 = {2.0, 2.0};
    const Point2d p8 = {2.0, 3.0};
    const auto result =
      within(ConvexPolygon2d({p1, p2, p3, p4}), ConvexPolygon2d({p5, p6, p7, p8}));

    EXPECT_FALSE(result);
  }

  {  // Both polygons are the same
    const Point2d p1 = {1.0, 1.0};
    const Point2d p2 = {1.0, -1.0};
    const Point2d p3 = {-1.0, -1.0};
    const Point2d p4 = {-1.0, 1.0};
    const Point2d p5 = {1.0, 1.0};
    const Point2d p6 = {1.0, -1.0};
    const Point2d p7 = {-1.0, -1.0};
    const Point2d p8 = {-1.0, 1.0};
    const auto result =
      within(ConvexPolygon2d({p1, p2, p3, p4}), ConvexPolygon2d({p5, p6, p7, p8}));

    EXPECT_TRUE(result);
  }
}

TEST(alt_geometry, areaRand)
{
  std::vector<autoware::universe_utils::Polygon2d> polygons;
  constexpr auto polygons_nb = 500;
  constexpr auto max_vertices = 10;
  constexpr auto max_values = 1000;

  autoware::universe_utils::StopWatch<std::chrono::nanoseconds, std::chrono::nanoseconds> sw;
  for (auto vertices = 3UL; vertices < max_vertices; ++vertices) {
    double ground_truth_area_ns = 0.0;
    double alt_area_ns = 0.0;

    polygons.clear();
    for (auto i = 0; i < polygons_nb; ++i) {
      polygons.push_back(autoware::universe_utils::random_convex_polygon(vertices, max_values));
    }
    for (auto i = 0UL; i < polygons.size(); ++i) {
      sw.tic();
      const auto ground_truth = boost::geometry::area(polygons[i]);
      ground_truth_area_ns += sw.toc();

      const auto alt_poly = autoware::universe_utils::alt::from_boost(polygons[i]);
      sw.tic();
      const auto alt = autoware::universe_utils::area(alt_poly);
      alt_area_ns += sw.toc();

      if (std::abs(alt - ground_truth) > epsilon) {
        std::cout << "Alt failed for the polygon: ";
        std::cout << boost::geometry::wkt(polygons[i]) << std::endl;
      }
      EXPECT_NEAR(ground_truth, alt, epsilon);
    }
    std::printf("polygons_nb = %d, vertices = %ld\n", polygons_nb, vertices);
    std::printf(
      "\tArea:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n", ground_truth_area_ns / 1e6,
      alt_area_ns / 1e6);
  }
}

TEST(alt_geometry, convexHullRand)
{
  std::vector<autoware::universe_utils::Polygon2d> polygons;
  constexpr auto polygons_nb = 500;
  constexpr auto max_vertices = 10;
  constexpr auto max_values = 1000;

  autoware::universe_utils::StopWatch<std::chrono::nanoseconds, std::chrono::nanoseconds> sw;
  for (auto vertices = 3UL; vertices < max_vertices; ++vertices) {
    double ground_truth_hull_ns = 0.0;
    double alt_hull_ns = 0.0;

    polygons.clear();
    for (auto i = 0; i < polygons_nb; ++i) {
      polygons.push_back(autoware::universe_utils::random_convex_polygon(vertices, max_values));
    }
    for (auto i = 0UL; i < polygons.size(); ++i) {
      autoware::universe_utils::MultiPoint2d outer;
      for (const auto & point : polygons[i].outer()) {
        outer.push_back(point);
      }
      autoware::universe_utils::Polygon2d ground_truth;
      sw.tic();
      boost::geometry::convex_hull(outer, ground_truth);
      ground_truth_hull_ns += sw.toc();

      const auto vertices = autoware::universe_utils::alt::from_boost(polygons[i]).vertices();
      sw.tic();
      const auto alt = autoware::universe_utils::convex_hull(vertices);
      alt_hull_ns += sw.toc();

      if (ground_truth.outer().size() - 1 != alt.vertices().size()) {
        std::cout << "Alt failed for the polygon: ";
        std::cout << boost::geometry::wkt(polygons[i]) << std::endl;
      }
      ASSERT_EQ(
        ground_truth.outer().size() - 1,
        alt.vertices().size());  // alt::ConvexPolygon2d does not have closing point
      for (size_t i = 0; i < alt.vertices().size(); ++i) {
        EXPECT_NEAR(ground_truth.outer().at(i).x(), alt.vertices().at(i).x(), epsilon);
        EXPECT_NEAR(ground_truth.outer().at(i).y(), alt.vertices().at(i).y(), epsilon);
      }
    }
    std::printf("polygons_nb = %d, vertices = %ld\n", polygons_nb, vertices);
    std::printf(
      "\tConvex Hull:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_hull_ns / 1e6, alt_hull_ns / 1e6);
  }
}

TEST(alt_geometry, coveredByRand)
{
  std::vector<autoware::universe_utils::Polygon2d> polygons;
  constexpr auto polygons_nb = 500;
  constexpr auto max_vertices = 10;
  constexpr auto max_values = 1000;

  autoware::universe_utils::StopWatch<std::chrono::nanoseconds, std::chrono::nanoseconds> sw;
  for (auto vertices = 3UL; vertices < max_vertices; ++vertices) {
    double ground_truth_covered_ns = 0.0;
    double ground_truth_not_covered_ns = 0.0;
    double alt_covered_ns = 0.0;
    double alt_not_covered_ns = 0.0;
    int covered_count = 0;

    polygons.clear();
    for (auto i = 0; i < polygons_nb; ++i) {
      polygons.push_back(autoware::universe_utils::random_convex_polygon(vertices, max_values));
    }
    for (auto i = 0UL; i < polygons.size(); ++i) {
      for (const auto & point : polygons[i].outer()) {
        for (auto j = 0UL; j < polygons.size(); ++j) {
          sw.tic();
          const auto ground_truth = boost::geometry::covered_by(point, polygons[j]);
          if (ground_truth) {
            ++covered_count;
            ground_truth_covered_ns += sw.toc();
          } else {
            ground_truth_not_covered_ns += sw.toc();
          }

          const auto alt_point = autoware::universe_utils::alt::from_boost(point);
          const auto alt_poly = autoware::universe_utils::alt::from_boost(polygons[j]);
          sw.tic();
          const auto alt = autoware::universe_utils::covered_by(alt_point, alt_poly);
          if (alt) {
            alt_covered_ns += sw.toc();
          } else {
            alt_not_covered_ns += sw.toc();
          }

          if (ground_truth != alt) {
            std::cout << "Alt failed for the point and polygon: ";
            std::cout << boost::geometry::wkt(point) << boost::geometry::wkt(polygons[j])
                      << std::endl;
          }
          EXPECT_EQ(ground_truth, alt);
        }
      }
    }
    std::printf(
      "polygons_nb = %d, vertices = %ld, %d / %ld covered pairs\n", polygons_nb, vertices,
      covered_count, polygons_nb * vertices * polygons_nb);
    std::printf(
      "\tCovered:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_covered_ns / 1e6, alt_covered_ns / 1e6);
    std::printf(
      "\tNot covered:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_not_covered_ns / 1e6, alt_not_covered_ns / 1e6);
    std::printf(
      "\tTotal:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      (ground_truth_not_covered_ns + ground_truth_covered_ns) / 1e6,
      (alt_not_covered_ns + alt_covered_ns) / 1e6);
  }
}

TEST(alt_geometry, disjointRand)
{
  std::vector<autoware::universe_utils::Polygon2d> polygons;
  constexpr auto polygons_nb = 500;
  constexpr auto max_vertices = 10;
  constexpr auto max_values = 1000;

  autoware::universe_utils::StopWatch<std::chrono::nanoseconds, std::chrono::nanoseconds> sw;
  for (auto vertices = 3UL; vertices < max_vertices; ++vertices) {
    double ground_truth_disjoint_ns = 0.0;
    double ground_truth_not_disjoint_ns = 0.0;
    double alt_disjoint_ns = 0.0;
    double alt_not_disjoint_ns = 0.0;
    int disjoint_count = 0;

    polygons.clear();
    for (auto i = 0; i < polygons_nb; ++i) {
      polygons.push_back(autoware::universe_utils::random_convex_polygon(vertices, max_values));
    }
    for (auto i = 0UL; i < polygons.size(); ++i) {
      for (auto j = 0UL; j < polygons.size(); ++j) {
        sw.tic();
        const auto ground_truth = boost::geometry::disjoint(polygons[i], polygons[j]);
        if (ground_truth) {
          ++disjoint_count;
          ground_truth_disjoint_ns += sw.toc();
        } else {
          ground_truth_not_disjoint_ns += sw.toc();
        }

        const auto alt_poly1 = autoware::universe_utils::alt::from_boost(polygons[i]);
        const auto alt_poly2 = autoware::universe_utils::alt::from_boost(polygons[j]);
        sw.tic();
        const auto alt = autoware::universe_utils::disjoint(alt_poly1, alt_poly2);
        if (alt) {
          alt_disjoint_ns += sw.toc();
        } else {
          alt_not_disjoint_ns += sw.toc();
        }

        if (ground_truth != alt) {
          std::cout << "Failed for the 2 polygons: ";
          std::cout << boost::geometry::wkt(polygons[i]) << boost::geometry::wkt(polygons[j])
                    << std::endl;
        }
        EXPECT_EQ(ground_truth, alt);
      }
    }
    std::printf(
      "polygons_nb = %d, vertices = %ld, %d / %d disjoint pairs\n", polygons_nb, vertices,
      disjoint_count, polygons_nb * polygons_nb);
    std::printf(
      "\tDisjoint:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_disjoint_ns / 1e6, alt_disjoint_ns / 1e6);
    std::printf(
      "\tNot disjoint:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_not_disjoint_ns / 1e6, alt_not_disjoint_ns / 1e6);
    std::printf(
      "\tTotal:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      (ground_truth_not_disjoint_ns + ground_truth_disjoint_ns) / 1e6,
      (alt_not_disjoint_ns + alt_disjoint_ns) / 1e6);
  }
}

TEST(alt_geometry, intersectsRand)
{
  std::vector<autoware::universe_utils::Polygon2d> polygons;
  constexpr auto polygons_nb = 500;
  constexpr auto max_vertices = 10;
  constexpr auto max_values = 1000;

  autoware::universe_utils::StopWatch<std::chrono::nanoseconds, std::chrono::nanoseconds> sw;
  for (auto vertices = 3UL; vertices < max_vertices; ++vertices) {
    double ground_truth_intersect_ns = 0.0;
    double ground_truth_no_intersect_ns = 0.0;
    double alt_intersect_ns = 0.0;
    double alt_no_intersect_ns = 0.0;
    int intersect_count = 0;

    polygons.clear();
    for (auto i = 0; i < polygons_nb; ++i) {
      polygons.push_back(autoware::universe_utils::random_convex_polygon(vertices, max_values));
    }
    for (auto i = 0UL; i < polygons.size(); ++i) {
      for (auto j = 0UL; j < polygons.size(); ++j) {
        sw.tic();
        const auto ground_truth = boost::geometry::intersects(polygons[i], polygons[j]);
        if (ground_truth) {
          ++intersect_count;
          ground_truth_intersect_ns += sw.toc();
        } else {
          ground_truth_no_intersect_ns += sw.toc();
        }

        const auto alt_poly1 = autoware::universe_utils::alt::from_boost(polygons[i]);
        const auto alt_poly2 = autoware::universe_utils::alt::from_boost(polygons[j]);
        sw.tic();
        const auto alt = autoware::universe_utils::intersects(alt_poly1, alt_poly2);
        if (alt) {
          alt_intersect_ns += sw.toc();
        } else {
          alt_no_intersect_ns += sw.toc();
        }

        if (ground_truth != alt) {
          std::cout << "Failed for the 2 polygons: ";
          std::cout << boost::geometry::wkt(polygons[i]) << boost::geometry::wkt(polygons[j])
                    << std::endl;
        }
        EXPECT_EQ(ground_truth, alt);
      }
    }
    std::printf(
      "polygons_nb = %d, vertices = %ld, %d / %d pairs with intersects\n", polygons_nb, vertices,
      intersect_count, polygons_nb * polygons_nb);
    std::printf(
      "\tIntersect:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_intersect_ns / 1e6, alt_intersect_ns / 1e6);
    std::printf(
      "\tNo intersect:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_no_intersect_ns / 1e6, alt_no_intersect_ns / 1e6);
    std::printf(
      "\tTotal:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      (ground_truth_no_intersect_ns + ground_truth_intersect_ns) / 1e6,
      (alt_no_intersect_ns + alt_intersect_ns) / 1e6);
  }
}

TEST(alt_geometry, touchesRand)
{
  std::vector<autoware::universe_utils::Polygon2d> polygons;
  constexpr auto polygons_nb = 500;
  constexpr auto max_vertices = 10;
  constexpr auto max_values = 1000;

  autoware::universe_utils::StopWatch<std::chrono::nanoseconds, std::chrono::nanoseconds> sw;
  for (auto vertices = 3UL; vertices < max_vertices; ++vertices) {
    double ground_truth_touching_ns = 0.0;
    double ground_truth_not_touching_ns = 0.0;
    double alt_touching_ns = 0.0;
    double alt_not_touching_ns = 0.0;
    int touching_count = 0;

    polygons.clear();
    for (auto i = 0; i < polygons_nb; ++i) {
      polygons.push_back(autoware::universe_utils::random_convex_polygon(vertices, max_values));
    }
    for (auto i = 0UL; i < polygons.size(); ++i) {
      for (const auto & point : polygons[i].outer()) {
        for (auto j = 0UL; j < polygons.size(); ++j) {
          sw.tic();
          const auto ground_truth = boost::geometry::touches(point, polygons[j]);
          if (ground_truth) {
            ++touching_count;
            ground_truth_touching_ns += sw.toc();
          } else {
            ground_truth_not_touching_ns += sw.toc();
          }

          const auto alt_point = autoware::universe_utils::alt::from_boost(point);
          const auto alt_poly = autoware::universe_utils::alt::from_boost(polygons[j]);
          sw.tic();
          const auto alt = autoware::universe_utils::touches(alt_point, alt_poly);
          if (alt) {
            alt_touching_ns += sw.toc();
          } else {
            alt_not_touching_ns += sw.toc();
          }

          if (ground_truth != alt) {
            std::cout << "Alt failed for the point and polygon: ";
            std::cout << boost::geometry::wkt(point) << boost::geometry::wkt(polygons[j])
                      << std::endl;
          }
          EXPECT_EQ(ground_truth, alt);
        }
      }
    }
    std::printf(
      "polygons_nb = %d, vertices = %ld, %d / %ld touching pairs\n", polygons_nb, vertices,
      touching_count, polygons_nb * vertices * polygons_nb);
    std::printf(
      "\tTouching:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_touching_ns / 1e6, alt_touching_ns / 1e6);
    std::printf(
      "\tNot touching:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_not_touching_ns / 1e6, alt_not_touching_ns / 1e6);
    std::printf(
      "\tTotal:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      (ground_truth_not_touching_ns + ground_truth_touching_ns) / 1e6,
      (alt_not_touching_ns + alt_touching_ns) / 1e6);
  }
}

TEST(alt_geometry, withinPolygonRand)
{
  std::vector<autoware::universe_utils::Polygon2d> polygons;
  constexpr auto polygons_nb = 500;
  constexpr auto max_vertices = 10;
  constexpr auto max_values = 1000;

  autoware::universe_utils::StopWatch<std::chrono::nanoseconds, std::chrono::nanoseconds> sw;
  for (auto vertices = 3UL; vertices < max_vertices; ++vertices) {
    double ground_truth_within_ns = 0.0;
    double ground_truth_not_within_ns = 0.0;
    double alt_within_ns = 0.0;
    double alt_not_within_ns = 0.0;
    int within_count = 0;

    polygons.clear();
    for (auto i = 0; i < polygons_nb; ++i) {
      polygons.push_back(autoware::universe_utils::random_convex_polygon(vertices, max_values));
    }
    for (auto i = 0UL; i < polygons.size(); ++i) {
      for (auto j = 0UL; j < polygons.size(); ++j) {
        sw.tic();
        const auto ground_truth = boost::geometry::within(polygons[i], polygons[j]);
        if (ground_truth) {
          ++within_count;
          ground_truth_within_ns += sw.toc();
        } else {
          ground_truth_not_within_ns += sw.toc();
        }

        const auto alt_poly1 = autoware::universe_utils::alt::from_boost(polygons[i]);
        const auto alt_poly2 = autoware::universe_utils::alt::from_boost(polygons[j]);
        sw.tic();
        const auto alt = autoware::universe_utils::within(alt_poly1, alt_poly2);
        if (alt) {
          alt_within_ns += sw.toc();
        } else {
          alt_not_within_ns += sw.toc();
        }

        if (ground_truth != alt) {
          std::cout << "Alt failed for the 2 polygons: ";
          std::cout << boost::geometry::wkt(polygons[i]) << boost::geometry::wkt(polygons[j])
                    << std::endl;
        }
        EXPECT_EQ(ground_truth, alt);
      }
    }
    std::printf(
      "polygons_nb = %d, vertices = %ld, %d / %d pairs either of which is within the other\n",
      polygons_nb, vertices, within_count, polygons_nb * polygons_nb);
    std::printf(
      "\tWithin:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_within_ns / 1e6, alt_within_ns / 1e6);
    std::printf(
      "\tNot within:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      ground_truth_not_within_ns / 1e6, alt_not_within_ns / 1e6);
    std::printf(
      "\tTotal:\n\t\tBoost::geometry = %2.2f ms\n\t\tAlt = %2.2f ms\n",
      (ground_truth_not_within_ns + ground_truth_within_ns) / 1e6,
      (alt_not_within_ns + alt_within_ns) / 1e6);
  }
}
