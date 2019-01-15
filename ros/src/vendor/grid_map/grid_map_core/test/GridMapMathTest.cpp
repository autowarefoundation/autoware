/*
 * GridMapMathTest.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/GridMapMath.hpp"

// Eigen
#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

// Vector
#include <vector>

using namespace std;
using namespace grid_map;

TEST(PositionFromIndex, Simple)
{
  Length mapLength(3.0, 2.0);
  Position mapPosition(-1.0, 2.0);
  double resolution = 1.0;
  Size bufferSize(3, 2);
  Position position;

  EXPECT_TRUE(getPositionFromIndex(position, Index(0, 0), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(1.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Index(1, 0), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Index(1, 1), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.5 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Index(2, 1), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(-1.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.5 + mapPosition.y(), position.y());

  EXPECT_FALSE(getPositionFromIndex(position, Index(3, 1), mapLength, mapPosition, resolution, bufferSize));
}

TEST(PositionFromIndex, CircularBuffer)
{
  Length mapLength(0.5, 0.4);
  Position mapPosition(-0.1, 13.4);
  double resolution = 0.1;
  Size bufferSize(5, 4);
  Index bufferStartIndex(3, 1);
  Position position;

  EXPECT_TRUE(getPositionFromIndex(position, Index(3, 1), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.2 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Index(4, 2), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.1 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(0.05 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Index(2, 0), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(-0.2 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Index(0, 0), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.0 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.15 + mapPosition.y(), position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Index(4, 3), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.1 + mapPosition.x(), position.x());
  EXPECT_DOUBLE_EQ(-0.05 + mapPosition.y(), position.y());

  EXPECT_FALSE(getPositionFromIndex(position, Index(5, 3), mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
}

TEST(IndexFromPosition, Simple)
{
  Length mapLength(3.0, 2.0);
  Position mapPosition(-12.4, -7.1);
  double resolution = 1.0;
  Index bufferSize(3, 2);
  Index index;

  EXPECT_TRUE(getIndexFromPosition(index, Position(1.0, 0.5) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Position(-1.0, -0.5) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(2, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Position(0.6, 0.1) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Position(0.4, -0.1) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Position(0.4, 0.1) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_FALSE(getIndexFromPosition(index, Position(4.0, 0.5) + mapPosition, mapLength, mapPosition, resolution, bufferSize));
}

TEST(IndexFromPosition, EdgeCases)
{
  Length mapLength(3.0, 2.0);
  Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Size bufferSize(3, 2);
  Index index;

  EXPECT_TRUE(getIndexFromPosition(index, Position(0.0, DBL_EPSILON), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Position(0.5 - DBL_EPSILON, -DBL_EPSILON), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Position(-0.5 - DBL_EPSILON, -DBL_EPSILON), mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(2, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_FALSE(getIndexFromPosition(index, Position(-1.5, 1.0), mapLength, mapPosition, resolution, bufferSize));
}

TEST(IndexFromPosition, CircularBuffer)
{
  Length mapLength(0.5, 0.4);
  Position mapPosition(0.4, -0.9);
  double resolution = 0.1;
  Size bufferSize(5, 4);
  Index bufferStartIndex(3, 1);
  Index index;

  EXPECT_TRUE(getIndexFromPosition(index, Position(0.2, 0.15) + mapPosition, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Position(0.03, -0.17) + mapPosition, mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));
}

TEST(checkIfPositionWithinMap, Inside)
{
  Length mapLength(50.0, 25.0);
  Position mapPosition(11.4, 0.0);

  EXPECT_TRUE(checkIfPositionWithinMap(Position(0.0, 0.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Position(5.0, 5.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Position(20.0, 10.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Position(20.0, -10.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Position(-20.0, 10.0) + mapPosition, mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Position(-20.0, -10.0) + mapPosition, mapLength, mapPosition));
}

TEST(checkIfPositionWithinMap, Outside)
{
  Length mapLength(10.0, 5.0);
  Position mapPosition(-3.0, 145.2);

  EXPECT_FALSE(checkIfPositionWithinMap(Position(5.5, 0.0) + mapPosition, mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Position(-5.5, 0.0) + mapPosition, mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Position(-5.5, 3.0) + mapPosition, mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Position(-5.5, -3.0) + mapPosition, mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Position(3.0, 3.0) + mapPosition, mapLength, mapPosition));
}

TEST(checkIfPositionWithinMap, EdgeCases)
{
  Length mapLength(2.0, 3.0);
  Position mapPosition(0.0, 0.0);

  EXPECT_FALSE(checkIfPositionWithinMap(Position(1.0, -1.5), mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Position(-1.0, 1.5), mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Position(1.0 + DBL_EPSILON, 1.0), mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Position((2.0 + DBL_EPSILON) / 2.0, 1.0), mapLength, mapPosition));
  EXPECT_FALSE(checkIfPositionWithinMap(Position(0.5, -1.5 - (2.0 * DBL_EPSILON)), mapLength, mapPosition));
  EXPECT_TRUE(checkIfPositionWithinMap(Position(-0.5, (3.0 + DBL_EPSILON) / 2.0), mapLength, mapPosition));
}

TEST(getIndexShiftFromPositionShift, All)
{
  double resolution = 1.0;
  Index indexShift;

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector(0.0, 0.0), resolution));
  EXPECT_EQ(0, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector(0.35, -0.45), resolution));
  EXPECT_EQ(0, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector(0.55, -0.45), resolution));
  EXPECT_EQ(-1, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector(-1.3, -2.65), resolution));
  EXPECT_EQ(1, indexShift(0));
  EXPECT_EQ(3, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector(-0.4, 0.09), 0.2));
  EXPECT_EQ(2, indexShift(0));
  EXPECT_EQ(0, indexShift(1));
}

TEST(getPositionShiftFromIndexShift, All)
{
  double resolution = 0.3;
  Vector positionShift;

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Index(0, 0), resolution));
  EXPECT_DOUBLE_EQ(0.0, positionShift.x());
  EXPECT_DOUBLE_EQ(0.0, positionShift.y());

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Index(1, -1), resolution));
  EXPECT_DOUBLE_EQ(-0.3, positionShift.x());
  EXPECT_DOUBLE_EQ(0.3, positionShift.y());

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Index(2, 1), resolution));
  EXPECT_DOUBLE_EQ(-0.6, positionShift.x());
  EXPECT_DOUBLE_EQ(-0.3, positionShift.y());
}

TEST(checkIfIndexInRange, All)
{
  Size bufferSize(10, 15);
  EXPECT_TRUE(checkIfIndexInRange(Index(0, 0), bufferSize));
  EXPECT_TRUE(checkIfIndexInRange(Index(9, 14), bufferSize));
  EXPECT_FALSE(checkIfIndexInRange(Index(10, 5), bufferSize));
  EXPECT_FALSE(checkIfIndexInRange(Index(5, 300), bufferSize));
  EXPECT_FALSE(checkIfIndexInRange(Index(-1, 0), bufferSize));
  EXPECT_FALSE(checkIfIndexInRange(Index(0, -300), bufferSize));
}

TEST(boundIndexToRange, All)
{
  int index;
  int bufferSize = 10;

  index = 0;
  boundIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 1;
  boundIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -1;
  boundIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 9;
  boundIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 10;
  boundIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 35;
  boundIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = -19;
  boundIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);
}

TEST(wrapIndexToRange, All)
{
  int index;
  int bufferSize = 10;

  index = 0;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 1;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -1;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 9;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 10;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 11;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = 35;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(5, index);

  index = -9;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -19;
  wrapIndexToRange(index, bufferSize);
  EXPECT_EQ(1, index);
}

TEST(boundPositionToRange, Simple)
{
  double epsilon = 11.0 * numeric_limits<double>::epsilon();

  Length mapLength(30.0, 10.0);
  Position mapPosition(0.0, 0.0);
  Position position;

  position << 0.0, 0.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.0, position.y());

  position << 15.0, 5.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 15.0 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 5.0 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -15.0, -5.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 15.0 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 5.0 * epsilon);
  EXPECT_LE(-5.0, position.y());

  position << 16.0, 6.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 16.0 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 6.0 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -16.0, -6.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 16.0 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 6.0 * epsilon);
  EXPECT_LE(-5.0, position.y());

  position << 1e6, 1e6;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(15.0, position.x(), 1e6 * epsilon);
  EXPECT_GE(15.0, position.x());
  EXPECT_NEAR(5.0, position.y(), 1e6 * epsilon);
  EXPECT_GE(5.0, position.y());

  position << -1e6, -1e6;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-15.0, position.x(), 1e6 * epsilon);
  EXPECT_LE(-15.0, position.x());
  EXPECT_NEAR(-5.0, position.y(), 1e6 * epsilon);
  EXPECT_LE(-5.0, position.y());
}

TEST(boundPositionToRange, Position)
{
  double epsilon = 11.0 * numeric_limits<double>::epsilon();

  Length mapLength(30.0, 10.0);
  Position mapPosition(1.0, 2.0);
  Position position;

  position << 0.0, 0.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.0, position.y());

  position << 16.0, 7.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 16.0 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 7.0 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -14.0, -3.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 14.0 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 3.0 * epsilon);
  EXPECT_LE(-3.0, position.y());

  position << 17.0, 8.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 17.0 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 8.0 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -15.0, -4.0;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 15.0 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 4.0 * epsilon);
  EXPECT_LE(-3.0, position.y());

  position << 1e6, 1e6;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(16.0, position.x(), 1e6 * epsilon);
  EXPECT_GE(16.0, position.x());
  EXPECT_NEAR(7.0, position.y(), 1e6 * epsilon);
  EXPECT_GE(7.0, position.y());

  position << -1e6, -1e6;
  boundPositionToRange(position, mapLength, mapPosition);
  EXPECT_NEAR(-14.0, position.x(), 1e6 * epsilon);
  EXPECT_LE(-14.0, position.x());
  EXPECT_NEAR(-3.0, position.y(), 1e6 * epsilon);
  EXPECT_LE(-3.0, position.y());
}

TEST(getSubmapInformation, Simple)
{
  // Map
  Length mapLength(5.0, 4.0);
  Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Size bufferSize(5, 4);

  // Requested submap
  Position requestedSubmapPosition;
  Position requestedSubmapLength;

  // The returned submap indeces
  Index submapTopLeftIndex;
  Index submapSize;
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  requestedSubmapPosition << 0.0, 0.5;
  requestedSubmapLength << 0.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                           requestedSubmapPosition, requestedSubmapLength, mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(3, submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.5, submapPosition.y());
  EXPECT_DOUBLE_EQ(1.0, submapLength(0));
  EXPECT_DOUBLE_EQ(3.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(1, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, Zero)
{
  // Map
  Length mapLength(5.0, 4.0);
  Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Size bufferSize(5, 4);

  // Requested submap
  Position requestedSubmapPosition;
  Length requestedSubmapLength;

  // The returned submap indeces
  Index submapTopLeftIndex;
  Index submapSize;
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  requestedSubmapPosition << -1.0, -0.5;
  requestedSubmapLength << 0.0, 0.0;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(3, submapTopLeftIndex(0));
  EXPECT_EQ(2, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(1, submapSize(1));
  EXPECT_DOUBLE_EQ(requestedSubmapPosition.x(), submapPosition.x());
  EXPECT_DOUBLE_EQ(requestedSubmapPosition.y(), submapPosition.y());
  EXPECT_DOUBLE_EQ(resolution, submapLength(0));
  EXPECT_DOUBLE_EQ(resolution, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, ExceedingBoundaries)
{
  // Map
  Length mapLength(5.0, 4.0);
  Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Size bufferSize(5, 4);

  // Requested submap
  Position requestedSubmapPosition;
  Length requestedSubmapLength;

  // The returned submap indeces
  Index submapTopLeftIndex;
  Size submapSize;
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  requestedSubmapPosition << 2.0, 1.5;
  requestedSubmapLength << 2.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(0, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(2, submapSize(0));
  EXPECT_EQ(2, submapSize(1));
  EXPECT_DOUBLE_EQ(1.5, submapPosition.x());
  EXPECT_DOUBLE_EQ(1.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(2.0, submapLength(0));
  EXPECT_DOUBLE_EQ(2.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));

  requestedSubmapPosition << 0.0, 0.0;
  requestedSubmapLength << 1e6, 1e6;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize));
  EXPECT_EQ(0, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(bufferSize(0), submapSize(0));
  EXPECT_EQ(bufferSize(1), submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(mapLength(0), submapLength(0));
  EXPECT_DOUBLE_EQ(mapLength(1), submapLength(1));
  EXPECT_EQ(2, requestedIndexInSubmap(0));
  EXPECT_LE(1, requestedIndexInSubmap(1));
  EXPECT_GE(2, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, CircularBuffer)
{
  // Map
  Length mapLength(5.0, 4.0);
  Position mapPosition(0.0, 0.0);
  double resolution = 1.0;
  Size bufferSize(5, 4);
  Index bufferStartIndex(2, 1);

  // Requested submap
  Position requestedSubmapPosition;
  Length requestedSubmapLength;

  // The returned submap indeces
  Index submapTopLeftIndex;
  Size submapSize;
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  requestedSubmapPosition << 0.0, 0.5;
  requestedSubmapLength << 0.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(4, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(3, submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.5, submapPosition.y());
  EXPECT_DOUBLE_EQ(1.0, submapLength(0));
  EXPECT_DOUBLE_EQ(3.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(1, requestedIndexInSubmap(1));

  requestedSubmapPosition << 2.0, 1.5;
  requestedSubmapLength << 2.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(2, submapSize(0));
  EXPECT_EQ(2, submapSize(1));
  EXPECT_DOUBLE_EQ(1.5, submapPosition.x());
  EXPECT_DOUBLE_EQ(1.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(2.0, submapLength(0));
  EXPECT_DOUBLE_EQ(2.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));

  requestedSubmapPosition << 0.0, 0.0;
  requestedSubmapLength << 1e6, 1e6;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(bufferSize(0), submapSize(0));
  EXPECT_EQ(bufferSize(1), submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(mapLength(0), submapLength(0));
  EXPECT_DOUBLE_EQ(mapLength(1), submapLength(1));
  EXPECT_EQ(2, requestedIndexInSubmap(0));
  EXPECT_LE(1, requestedIndexInSubmap(1));
  EXPECT_GE(2, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, Debug1)
{
  // Map
  Length mapLength(4.98, 4.98);
  Position mapPosition(-4.98, -5.76);
  double resolution = 0.06;
  Size bufferSize(83, 83);
  Index bufferStartIndex(0, 13);

  // Requested submap
  Position requestedSubmapPosition(-7.44, -3.42);
  Length requestedSubmapLength(0.12, 0.12);

  // The returned submap indeces
  Index submapTopLeftIndex;
  Size submapSize;
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, submapSize(0));
  EXPECT_EQ(3, submapSize(1));
  EXPECT_DOUBLE_EQ(0.12, submapLength(0));
  EXPECT_DOUBLE_EQ(0.18, submapLength(1));
}

TEST(getSubmapInformation, Debug2)
{
  // Map
  Length mapLength(4.98, 4.98);
  Position mapPosition(2.46, -25.26);
  double resolution = 0.06;
  Size bufferSize(83, 83);
  Index bufferStartIndex(42, 6);

  // Requested submap
  Position requestedSubmapPosition(0.24, -26.82);
  Length requestedSubmapLength(0.624614, 0.462276);

  // The returned submap indeces
  Index submapTopLeftIndex;
  Size submapSize;
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, mapPosition, resolution, bufferSize, bufferStartIndex));
  EXPECT_LT(0, submapSize(0));
  EXPECT_LT(0, submapSize(1));
  EXPECT_LT(0.0, submapLength(0));
  EXPECT_LT(0.0, submapLength(1));
}

TEST(getBufferRegionsForSubmap, Trivial)
{
  Size bufferSize(5, 4);
  Index submapIndex(0, 0);
  Size submapSize(0, 0);
  std::vector<BufferRegion> regions;

  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
  EXPECT_EQ(1, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(0, regions[0].getStartIndex()[0]);
  EXPECT_EQ(0, regions[0].getStartIndex()[1]);
  EXPECT_EQ(0, regions[0].getSize()[0]);
  EXPECT_EQ(0, regions[0].getSize()[1]);

  submapSize << 0, 7;
  EXPECT_FALSE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));

  submapSize << 6, 7;
  EXPECT_FALSE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
}

TEST(getBufferRegionsForSubmap, Simple)
{
  Size bufferSize(5, 4);
  Index submapIndex(1, 2);
  Size submapSize(3, 2);
  std::vector<BufferRegion> regions;

  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize));
  EXPECT_EQ(1, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(1, regions[0].getStartIndex()[0]);
  EXPECT_EQ(2, regions[0].getStartIndex()[1]);
  EXPECT_EQ(3, regions[0].getSize()[0]);
  EXPECT_EQ(2, regions[0].getSize()[1]);
}

TEST(getBufferRegionsForSubmap, CircularBuffer)
{
  Size bufferSize(5, 4);
  Index submapIndex;
  Size submapSize;
  Index bufferStartIndex(3, 1);
  std::vector<BufferRegion> regions;

  submapIndex << 3, 1;
  submapSize << 2, 3;
  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(3, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);

  submapIndex << 4, 1;
  submapSize << 2, 3;
  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(4, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(1, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);
  EXPECT_EQ(BufferRegion::Quadrant::BottomLeft, regions[1].getQuadrant());
  EXPECT_EQ(0, regions[1].getStartIndex()[0]);
  EXPECT_EQ(1, regions[1].getStartIndex()[1]);
  EXPECT_EQ(1, regions[1].getSize()[0]);
  EXPECT_EQ(3, regions[1].getSize()[1]);

  submapIndex << 1, 0;
  submapSize << 2, 1;
  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::BottomRight, regions[0].getQuadrant());
  EXPECT_EQ(1, regions[0].getStartIndex()[0]);
  EXPECT_EQ(0, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(1, regions[0].getSize()[1]);

  submapIndex << 3, 1;
  submapSize << 5, 4;
  EXPECT_TRUE(getBufferRegionsForSubmap(regions, submapIndex, submapSize, bufferSize, bufferStartIndex));\
  EXPECT_EQ(4, regions.size());
  EXPECT_EQ(BufferRegion::Quadrant::TopLeft, regions[0].getQuadrant());
  EXPECT_EQ(3, regions[0].getStartIndex()[0]);
  EXPECT_EQ(1, regions[0].getStartIndex()[1]);
  EXPECT_EQ(2, regions[0].getSize()[0]);
  EXPECT_EQ(3, regions[0].getSize()[1]);
  EXPECT_EQ(BufferRegion::Quadrant::TopRight, regions[1].getQuadrant());
  EXPECT_EQ(3, regions[1].getStartIndex()[0]);
  EXPECT_EQ(0, regions[1].getStartIndex()[1]);
  EXPECT_EQ(2, regions[1].getSize()[0]);
  EXPECT_EQ(1, regions[1].getSize()[1]);
  EXPECT_EQ(BufferRegion::Quadrant::BottomLeft, regions[2].getQuadrant());
  EXPECT_EQ(0, regions[2].getStartIndex()[0]);
  EXPECT_EQ(1, regions[2].getStartIndex()[1]);
  EXPECT_EQ(3, regions[2].getSize()[0]);
  EXPECT_EQ(3, regions[2].getSize()[1]);
  EXPECT_EQ(BufferRegion::Quadrant::BottomRight, regions[3].getQuadrant());
  EXPECT_EQ(0, regions[3].getStartIndex()[0]);
  EXPECT_EQ(0, regions[3].getStartIndex()[1]);
  EXPECT_EQ(3, regions[3].getSize()[0]);
  EXPECT_EQ(1, regions[3].getSize()[1]);
}

TEST(checkIncrementIndex, Simple)
{
  Index index(0, 0);
  Size bufferSize(4, 3);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(1, index[1]);

  for (int i = 0; i < 6; i++) {
    EXPECT_TRUE(incrementIndex(index, bufferSize));
  }
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_FALSE(incrementIndex(index, bufferSize));
  EXPECT_EQ(index[0], index[0]);
  EXPECT_EQ(index[1], index[1]);
}

TEST(checkIncrementIndex, CircularBuffer)
{
  Size bufferSize(4, 3);
  Index bufferStartIndex(2, 1);
  Index index(bufferStartIndex);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_FALSE(incrementIndex(index, bufferSize, bufferStartIndex));
  EXPECT_EQ(index[0], index[0]);
  EXPECT_EQ(index[1], index[1]);
}

TEST(checkIncrementIndexForSubmap, Simple)
{
  Index submapIndex(0, 0);
  Index index;
  Index submapTopLeftIndex(3, 1);
  Size submapBufferSize(2, 4);
  Size bufferSize(8, 5);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(1, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(2, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(2, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(3, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(3, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(0, submapIndex[1]);
  EXPECT_EQ(4, index[0]);
  EXPECT_EQ(1, index[1]);

  submapIndex << 1, 2;
  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(4, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_FALSE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));

  submapIndex << 2, 0;
  EXPECT_FALSE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize));
}

TEST(checkIncrementIndexForSubmap, CircularBuffer)
{
  Index submapIndex(0, 0);
  Index index;
  Index submapTopLeftIndex(6, 3);
  Size submapBufferSize(2, 4);
  Size bufferSize(8, 5);
  Index bufferStartIndex(3, 2);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(1, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(4, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(2, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(0, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(6, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(0, submapIndex[1]);
  EXPECT_EQ(7, index[0]);
  EXPECT_EQ(3, index[1]);

  submapIndex << 1, 2;
  EXPECT_TRUE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(1, submapIndex[0]);
  EXPECT_EQ(3, submapIndex[1]);
  EXPECT_EQ(7, index[0]);
  EXPECT_EQ(1, index[1]);

  EXPECT_FALSE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));

  submapIndex << 2, 0;
  EXPECT_FALSE(incrementIndexForSubmap(submapIndex, index, submapTopLeftIndex, submapBufferSize, bufferSize, bufferStartIndex));
}

TEST(getIndexFromLinearIndex, Simple)
{
  EXPECT_TRUE((Index(0, 0) == getIndexFromLinearIndex(0, Size(8, 5), false)).all());
  EXPECT_TRUE((Index(1, 0) == getIndexFromLinearIndex(1, Size(8, 5), false)).all());
  EXPECT_TRUE((Index(0, 1) == getIndexFromLinearIndex(1, Size(8, 5), true)).all());
  EXPECT_TRUE((Index(2, 0) == getIndexFromLinearIndex(2, Size(8, 5), false)).all());
  EXPECT_TRUE((Index(0, 1) == getIndexFromLinearIndex(8, Size(8, 5), false)).all());
  EXPECT_TRUE((Index(7, 4) == getIndexFromLinearIndex(39, Size(8, 5), false)).all());
}
