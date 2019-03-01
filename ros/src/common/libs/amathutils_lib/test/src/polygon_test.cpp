/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Yukihiro Saito
 */

#include <gtest/gtest.h>
#include <vector>
#include "amathutils_lib/polygon.hpp"
#include "amathutils_lib/vector2d.hpp"

using namespace amathutils;
class PolygonTestSuite : public ::testing::Test
{
  public:
    PolygonTestSuite() {}
    ~PolygonTestSuite() {}
};

// static bool isInPolygon(const std::vector<Vector2d> &v_point, const Vector2d &point);
// void setPolygon(const std::vector<Vector2d> &v_point);
// bool isInPolygon(const Vector2d &point);
TEST(TestSuite, CheckSimple)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;
    points.push_back(Vector2d(0, 0));
    points.push_back(Vector2d(10, 0));
    points.push_back(Vector2d(10, 10));
    points.push_back(Vector2d(0, 10));

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5)), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5)), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(-5, 0), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(-5, 0), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, -5), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, -5), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(15, 0), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(15, 0), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, -5), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, -5), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(15, 10), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(15, 10), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(15, 10), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(15, 10), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, -5), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, -5), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 15), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 15), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(-5, 0), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(-5, 0), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, -5), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, -5), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(15, 0), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(15, 0), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, -5), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, -5), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(15, 10), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(15, 10), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(15, 10), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(15, 10), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, -5), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, -5), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 15), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 15), false), false);
}

TEST(TestSuite, CheckSimpleInverse)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;
    points.push_back(Vector2d(0, 0));
    points.push_back(Vector2d(0, 10));
    points.push_back(Vector2d(10, 10));
    points.push_back(Vector2d(10, 0));

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5)), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5)), true);
}

TEST(TestSuite, CheckLoopPoint)
{
    {
        Polygon2d polygon_test;
        std::vector<Vector2d> points;
        points.push_back(Vector2d(0, 0));
        points.push_back(Vector2d(10, 0));
        points.push_back(Vector2d(10, 10));
        points.push_back(Vector2d(0, 10));
        points.push_back(Vector2d(0, 0));

        polygon_test.setPolygon(points);

        ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5)), true);
        ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5)), true);
    }
    {
        Polygon2d polygon_test;
        std::vector<Vector2d> points;
        points.push_back(Vector2d(10, 0));
        points.push_back(Vector2d(10, 10));
        points.push_back(Vector2d(0, 10));
        points.push_back(Vector2d(0, 0));
        points.push_back(Vector2d(10, 0));

        polygon_test.setPolygon(points);

        ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5)), true);
        ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5)), true);
    }
    {
        Polygon2d polygon_test;
        std::vector<Vector2d> points;
        points.push_back(Vector2d(10, 10));
        points.push_back(Vector2d(0, 10));
        points.push_back(Vector2d(0, 0));
        points.push_back(Vector2d(10, 0));
        points.push_back(Vector2d(10, 10));

        polygon_test.setPolygon(points);

        ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5)), true);
        ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5)), true);
    }
    {
        Polygon2d polygon_test;
        std::vector<Vector2d> points;
        points.push_back(Vector2d(0, 10));
        points.push_back(Vector2d(0, 0));
        points.push_back(Vector2d(10, 0));
        points.push_back(Vector2d(10, 10));
        points.push_back(Vector2d(0, 10));

        polygon_test.setPolygon(points);

        ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5)), true);
        ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5)), true);
    }
}

TEST(TestSuite, CheckEmpty)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5)), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5)), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 0)), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 0)), false);
}

TEST(TestSuite, CheckOnePoint)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;
    points.push_back(Vector2d(0, 0));

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5)), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5)), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 0)), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 0)), false);
}

TEST(TestSuite, CheckTwoPoint)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;
    points.push_back(Vector2d(0, 0));
    points.push_back(Vector2d(10, 10));

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5), true), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5), true), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 5), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 5), false), false);
}

TEST(TestSuite, CheckBoundary)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;
    points.push_back(Vector2d(0, 0));
    points.push_back(Vector2d(10, 0));
    points.push_back(Vector2d(10, 10));
    points.push_back(Vector2d(0, 10));

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 5), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 5), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 5), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 5), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 10), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 10), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 0), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 0), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 5), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 5), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 5), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 5), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 10), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 10), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 0), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 0), false), false);
}

TEST(TestSuite, CheckBoundaryInverse)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;
    points.push_back(Vector2d(0, 0));
    points.push_back(Vector2d(0, 10));
    points.push_back(Vector2d(10, 10));
    points.push_back(Vector2d(10, 0));

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 5), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 5), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 5), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 5), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 10), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 10), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 0), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 0), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 5), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 5), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 5), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 5), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 10), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 10), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(5, 0), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(5, 0), false), false);
}

TEST(TestSuite, CheckCorner)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;
    points.push_back(Vector2d(0, 0));
    points.push_back(Vector2d(10, 0));
    points.push_back(Vector2d(10, 10));
    points.push_back(Vector2d(0, 10));

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 0), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 0), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 0), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 0), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 10), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 10), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 10), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 10), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 0), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 0), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 0), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 0), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 10), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 10), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 10), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 10), false), false);
}

TEST(TestSuite, CheckCornerInverse)
{
    Polygon2d polygon_test;
    std::vector<Vector2d> points;
    points.push_back(Vector2d(0, 0));
    points.push_back(Vector2d(0, 10));
    points.push_back(Vector2d(10, 10));
    points.push_back(Vector2d(10, 0));

    polygon_test.setPolygon(points);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 0), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 0), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 0), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 0), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 10), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 10), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 10), true), true);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 10), true), true);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 0), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 0), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 0), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 0), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(10, 10), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(10, 10), false), false);

    ASSERT_EQ(polygon_test.isInPolygon(Vector2d(0, 10), false), false);
    ASSERT_EQ(polygon_test.isInPolygon(points, Vector2d(0, 10), false), false);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
