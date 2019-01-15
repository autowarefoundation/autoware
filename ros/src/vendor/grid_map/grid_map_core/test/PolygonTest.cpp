/*
 * PolygonTest.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Martin Wermelinger, Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/Polygon.hpp"

// gtest
#include <gtest/gtest.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace grid_map;

TEST(Polygon, getCentroidTriangle)
{
  Polygon triangle;
  triangle.addVertex(Vector2d(0.0, 0.0));
  triangle.addVertex(Vector2d(1.0, 0.0));
  triangle.addVertex(Vector2d(0.5, 1.0));

  Position expectedCentroid;
  expectedCentroid.x() = 1.0 / 3.0 * (1.0 + 0.5);
  expectedCentroid.y() = 1.0 / 3.0;
  Position centroid = triangle.getCentroid();
  EXPECT_DOUBLE_EQ(expectedCentroid.x(), centroid.x());
  EXPECT_DOUBLE_EQ(expectedCentroid.y(), centroid.y());
}

TEST(Polygon, getCentroidRectangle)
{
  Polygon rectangle;
  rectangle.addVertex(Vector2d(-2.0, -1.0));
  rectangle.addVertex(Vector2d(-2.0, 2.0));
  rectangle.addVertex(Vector2d(1.0, 2.0));
  rectangle.addVertex(Vector2d(1.0, -1.0));

  Position expectedCentroid(-0.5, 0.5);
  Position centroid = rectangle.getCentroid();
  EXPECT_DOUBLE_EQ(expectedCentroid.x(), centroid.x());
  EXPECT_DOUBLE_EQ(expectedCentroid.y(), centroid.y());
}

TEST(Polygon, getBoundingBox)
{
  Polygon triangle;
  triangle.addVertex(Vector2d(0.0, 0.0));
  triangle.addVertex(Vector2d(0.5, -1.2));
  triangle.addVertex(Vector2d(1.0, 0.0));

  Position expectedCenter(0.5, -0.6);
  Length expectedLength(1.0, 1.2);
  Position center;
  Length length;
  triangle.getBoundingBox(center, length);

  EXPECT_DOUBLE_EQ(expectedCenter.x(), center.x());
  EXPECT_DOUBLE_EQ(expectedCenter.y(), center.y());
  EXPECT_DOUBLE_EQ(expectedLength.x(), length.x());
  EXPECT_DOUBLE_EQ(expectedLength.y(), length.y());
}

TEST(Polygon, convexHullPolygon)
{
  Polygon polygon1;
  polygon1.addVertex(Vector2d(0.0, 0.0));
  polygon1.addVertex(Vector2d(1.0, 1.0));
  polygon1.addVertex(Vector2d(0.0, 1.0));
  polygon1.addVertex(Vector2d(1.0, 0.0));

  Polygon polygon2;
  polygon2.addVertex(Vector2d(0.5, 0.5));
  polygon2.addVertex(Vector2d(0.5, 1.5));
  polygon2.addVertex(Vector2d(1.5, 0.5));
  polygon2.addVertex(Vector2d(1.5, 1.5));

  Polygon hull = Polygon::convexHull(polygon1, polygon2);

  EXPECT_EQ(6, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.5)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.01, 1.49)));
}

TEST(Polygon, convexHullCircles)
{
  Position center1(0.0, 0.0);
  Position center2(1.0, 0.0);
  double radius = 0.5;
  const int nVertices = 15;

  Polygon hull = Polygon::convexHullOfTwoCircles(center1, center2, radius);
  EXPECT_EQ(20, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.6)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.5, 0.2)));

  hull = Polygon::convexHullOfTwoCircles(center1, center2, radius, nVertices);
  EXPECT_EQ(nVertices + 1, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.6)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.5, 0.2)));

  hull = Polygon::convexHullOfTwoCircles(center1, center1, radius);
  EXPECT_EQ(20, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.0, 0.25)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.0, -0.25)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.5)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.6, 0.0)));
  EXPECT_FALSE(hull.isInside(Vector2d(-0.6, 0.0)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.0, 0.6)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.0, -0.6)));

  hull = Polygon::convexHullOfTwoCircles(center1, center1, radius, nVertices);
  EXPECT_EQ(nVertices, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.0, 0.25)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.0, -0.25)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.5)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.6, 0.0)));
  EXPECT_FALSE(hull.isInside(Vector2d(-0.6, 0.0)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.0, 0.6)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.0, -0.6)));
}

TEST(Polygon, convexHullCircle)
{
  Position center(0.0, 0.0);
  double radius = 0.5;
  const int nVertices = 15;

  Polygon hull = Polygon::fromCircle(center, radius);

  EXPECT_EQ(20, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.49, 0.0)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.0, 0.0)));

  hull = Polygon::fromCircle(center, radius, nVertices);
  EXPECT_EQ(nVertices, hull.nVertices());
  EXPECT_TRUE(hull.isInside(Vector2d(-0.25, 0.0)));
  EXPECT_TRUE(hull.isInside(Vector2d(0.49, 0.0)));
  EXPECT_FALSE(hull.isInside(Vector2d(0.5, 0.4)));
  EXPECT_FALSE(hull.isInside(Vector2d(1.0, 0.0)));
}

TEST(convertToInequalityConstraints, triangle1)
{
  Polygon polygon({Position(1.0, 1.0), Position(0.0, 0.0), Position(1.1, -1.1)});
  MatrixXd A;
  VectorXd b;
  ASSERT_TRUE(polygon.convertToInequalityConstraints(A, b));
  EXPECT_NEAR(-1.3636, A(0, 0), 1e-4);
  EXPECT_NEAR( 1.3636, A(0, 1), 1e-4);
  EXPECT_NEAR(-1.5000, A(1, 0), 1e-4);
  EXPECT_NEAR(-1.5000, A(1, 1), 1e-4);
  EXPECT_NEAR( 2.8636, A(2, 0), 1e-4);
  EXPECT_NEAR( 0.1364, A(2, 1), 1e-4);
  EXPECT_NEAR( 0.0000, b(0), 1e-4);
  EXPECT_NEAR( 0.0000, b(1), 1e-4);
  EXPECT_NEAR( 3.0000, b(2), 1e-4);
}

TEST(convertToInequalityConstraints, triangle2)
{
  Polygon polygon({Position(-1.0, 0.5), Position(-1.0, -0.5), Position(1.0, -0.5)});
  MatrixXd A;
  VectorXd b;
  ASSERT_TRUE(polygon.convertToInequalityConstraints(A, b));
  EXPECT_NEAR(-1.5000, A(0, 0), 1e-4);
  EXPECT_NEAR( 0.0000, A(0, 1), 1e-4);
  EXPECT_NEAR( 0.0000, A(1, 0), 1e-4);
  EXPECT_NEAR(-3.0000, A(1, 1), 1e-4);
  EXPECT_NEAR( 1.5000, A(2, 0), 1e-4);
  EXPECT_NEAR( 3.0000, A(2, 1), 1e-4);
  EXPECT_NEAR( 1.5000, b(0), 1e-4);
  EXPECT_NEAR( 1.5000, b(1), 1e-4);
  EXPECT_NEAR( 0.0000, b(2), 1e-4);
}

TEST(offsetInward, triangle)
{
  Polygon polygon({Position(1.0, 1.0), Position(0.0, 0.0), Position(1.0, -1.0)});
  polygon.offsetInward(0.1);
  EXPECT_NEAR(0.9, polygon.getVertex(0)(0), 1e-4);
  EXPECT_NEAR(0.758579, polygon.getVertex(0)(1), 1e-4);
  EXPECT_NEAR(0.141421, polygon.getVertex(1)(0), 1e-4);
  EXPECT_NEAR(0.0, polygon.getVertex(1)(1), 1e-4);
  EXPECT_NEAR(0.9, polygon.getVertex(2)(0), 1e-4);
  EXPECT_NEAR(-0.758579, polygon.getVertex(2)(1), 1e-4);
}

TEST(triangulation, triangle)
{
  Polygon polygon({Position(1.0, 1.0), Position(0.0, 0.0), Position(1.0, -1.0)});
  std::vector<Polygon> polygons;
  polygons = polygon.triangulate();
  ASSERT_EQ(1, polygons.size());
  EXPECT_EQ(polygon.getVertex(0).x(), polygons[0].getVertex(0).x());
  EXPECT_EQ(polygon.getVertex(0).y(), polygons[0].getVertex(0).y());
  EXPECT_EQ(polygon.getVertex(1).x(), polygons[0].getVertex(1).x());
  EXPECT_EQ(polygon.getVertex(1).y(), polygons[0].getVertex(1).y());
  EXPECT_EQ(polygon.getVertex(2).x(), polygons[0].getVertex(2).x());
  EXPECT_EQ(polygon.getVertex(2).y(), polygons[0].getVertex(2).y());
}

TEST(triangulation, rectangle)
{
  Polygon rectangle;
  rectangle.addVertex(Vector2d(-2.0, -1.0));
  rectangle.addVertex(Vector2d(-2.0, 2.0));
  rectangle.addVertex(Vector2d(1.0, 2.0));
  rectangle.addVertex(Vector2d(1.0, -1.0));
  std::vector<Polygon> polygons;
  polygons = rectangle.triangulate();
  ASSERT_EQ(2, polygons.size());
  // TODO Extend.
}
