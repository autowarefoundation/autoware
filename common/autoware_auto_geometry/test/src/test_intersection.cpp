// Copyright 2021 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "autoware_auto_geometry/convex_hull.hpp"
#include "autoware_auto_geometry/intersection.hpp"

#include <gtest/gtest.h>

#include <list>

struct TestPoint
{
  autoware::common::types::float32_t x;
  autoware::common::types::float32_t y;
};

struct IntersectionTestParams
{
  std::list<TestPoint> polygon1_pts;
  std::list<TestPoint> polygon2_pts;
  std::list<TestPoint> expected_intersection_pts;
};

void order_ccw(std::list<TestPoint> & points)
{
  const auto end_it = autoware::common::geometry::convex_hull(points);
  ASSERT_EQ(end_it, points.end());  // Points should have represent a shape
}

class IntersectionTest : public ::testing::TestWithParam<IntersectionTestParams>
{
};

TEST_P(IntersectionTest, Basic)
{
  const auto get_ordered_polygon = [](auto polygon) {
    order_ccw(polygon);
    return polygon;
  };

  const auto polygon1 = get_ordered_polygon(GetParam().polygon1_pts);
  const auto polygon2 = get_ordered_polygon(GetParam().polygon2_pts);
  const auto expected_intersection = get_ordered_polygon(GetParam().expected_intersection_pts);

  const auto result = autoware::common::geometry::convex_polygon_intersection2d(polygon1, polygon2);

  ASSERT_EQ(result.size(), expected_intersection.size());
  auto expected_shape_it = expected_intersection.begin();
  for (auto result_it = result.begin(); result_it != result.end(); ++result_it) {
    EXPECT_FLOAT_EQ(result_it->x, expected_shape_it->x);
    EXPECT_FLOAT_EQ(result_it->y, expected_shape_it->y);
    ++expected_shape_it;
  }
}

INSTANTIATE_TEST_SUITE_P(
  Basic, IntersectionTest,
  ::testing::Values(
    IntersectionTestParams{{}, {}, {}},
    IntersectionTestParams{// Partial intersection
                           {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
                           {{5.0F, 5.0F}, {15.0F, 5.0F}, {5.0F, 15.0F}, {15.0F, 15.0F}},
                           {{5.0F, 5.0F}, {10.0F, 5.0F}, {5.0F, 10.0F}, {10.0F, 10.0F}}},
    // Full intersection with overlapping edges
    // TODO(yunus.caliskan): enable after #1231
    //        IntersectionTestParams{
    //            {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
    //            {{5.0F, 5.0F}, {10.0F, 5.0F}, {5.0F, 10.0F}, {10.0F, 10.0F}},
    //            {{5.0F, 5.0F}, {10.0F, 5.0F}, {5.0F, 10.0F}, {10.0F, 10.0F}},
    //        },
    IntersectionTestParams{
      // Fully contained
      {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
      {{5.0F, 5.0F}, {6.0F, 5.0F}, {5.0F, 7.0F}, {8.0F, 8.0F}},
      {{5.0F, 5.0F}, {6.0F, 5.0F}, {5.0F, 7.0F}, {8.0F, 8.0F}},
    },
    IntersectionTestParams{
      // Fully contained triangle
      {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
      {{5.0F, 5.0F}, {6.0F, 5.0F}, {5.0F, 7.0F}},
      {{5.0F, 5.0F}, {6.0F, 5.0F}, {5.0F, 7.0F}},
    },
    IntersectionTestParams{// Triangle rectangle intersection.
                           {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
                           {{5.0F, 1.0F}, {5.0F, 9.0F}, {15.0F, 5.0F}},
                           {{5.0F, 1.0F}, {5.0F, 9.0F}, {10.0F, 3.0F}, {10.0F, 7.0F}}},
    IntersectionTestParams{// No intersection
                           {{0.0F, 0.0F}, {10.0F, 0.0F}, {0.0F, 10.0F}, {10.0F, 10.0F}},
                           {{15.0F, 15.0F}, {20.0F, 15.0F}, {15.0F, 20.0F}, {20.0F, 20.0F}},
                           {}}  // cppcheck-suppress syntaxError
    ));

TEST(PolygonPointTest, Basic)
{
  GTEST_SKIP();  // TODO(yunus.caliskan): enable after #1231
  std::list<TestPoint> polygon{{5.0F, 5.0F}, {10.0F, 5.0F}, {5.0F, 10.0F}, {10.0F, 10.0F}};
  order_ccw(polygon);
  EXPECT_FALSE(autoware::common::geometry::is_point_inside_polygon_2d(
    polygon.begin(), polygon.end(), TestPoint{0.0F, 10.0F}));
}

// IoU of two intersecting shapes: a pentagon and a square. The test includes pen and paper
// computations for the intermediate steps as assertions.
TEST(IoUTest, PentagonRectangleIntersection)
{
  std::list<TestPoint> polygon1{
    {0.0F, 3.0F}, {3.0F, 4.0F}, {6.0F, 3.0F}, {4.0F, 1.0F}, {2.0F, 1.0F}};
  std::list<TestPoint> polygon2{{3.0F, 0.0F}, {3.0F, 2.0F}, {5.0F, 2.0F}, {5.0F, 0.0F}};

  order_ccw(polygon1);
  order_ccw(polygon2);

  const auto intersection =
    autoware::common::geometry::convex_polygon_intersection2d(polygon1, polygon2);
  const auto expected_intersection_area =
    autoware::common::geometry::area_2d(intersection.begin(), intersection.end());
  ASSERT_FLOAT_EQ(expected_intersection_area, 1.5F);  // Pen & paper proof.

  const auto expected_union_area =
    autoware::common::geometry::area_2d(polygon1.begin(), polygon1.end()) +
    autoware::common::geometry::area_2d(polygon2.begin(), polygon2.end()) -
    expected_intersection_area;
  ASSERT_FLOAT_EQ(expected_union_area, (11.0F + 4.0F - 1.5F));  // Pen & paper proof.

  const auto computed_iou =
    autoware::common::geometry::convex_intersection_over_union_2d(polygon1, polygon2);

  EXPECT_FLOAT_EQ(computed_iou, expected_intersection_area / expected_union_area);
}

// IoU of two non-intersecting rectangles.
TEST(IoUTest, NoIntersection)
{
  std::list<TestPoint> polygon1{{0.0F, 0.0F}, {0.0F, 1.0F}, {1.0F, 1.0F}, {1.0F, 0.0F}};
  std::list<TestPoint> polygon2{{3.0F, 0.0F}, {3.0F, 2.0F}, {5.0F, 2.0F}, {5.0F, 0.0F}};

  order_ccw(polygon1);
  order_ccw(polygon2);

  EXPECT_FLOAT_EQ(
    autoware::common::geometry::convex_intersection_over_union_2d(polygon1, polygon2), 0.0F);
}
