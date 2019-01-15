/*
 * LineIteratorTest.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: Dominic Jud
 *	 Institute: ETH Zurich, Robotic Systems Lab
 */

#include "grid_map_core/iterators/LineIterator.hpp"
#include "grid_map_core/GridMap.hpp"

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

using namespace grid_map;

TEST(LineIterator, StartOutsideMap)
{
  GridMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  LineIterator iterator(map, Position(2.0, 2.0), Position(0.0, 0.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, EndOutsideMap)
{
  GridMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  LineIterator iterator(map, Position(0.0, 0.0), Position(9.0, 6.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMap)
{
  GridMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  LineIterator iterator(map, Position(-7.0, -9.0), Position(8.0, 8.0));

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  ++iterator;
  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMapWithoutIntersectingMap)
{
  GridMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));

  LineIterator iterator(map, Position(-8.0, 8.0), Position(8.0, 8.0));

  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, MovedMap)
{
  GridMap map( { "types" });
  map.setGeometry(Length(7.0, 5.0), 1.0, Position(0.0, 0.0));
  map.move(Position(2.0, 2.0));

  LineIterator iterator(map, Position(0.0, 0.0), Position(2.0, 2.0));
  Position point;

  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(0, point.x());
  EXPECT_EQ(0, point.y());

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(1, point.x());
  EXPECT_EQ(1, point.y());

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(2, point.x());
  EXPECT_EQ(2, point.y());

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(LineIterator, StartAndEndOutsideMovedMap)
{
  GridMap map( { "types" });
  map.setGeometry(Length(7.0, 5.0), 1.0, Position(0.0, 0.0));
  map.move(Position(2.0, 2.0));

  LineIterator iterator(map, Position(0.0, 0.0), Position(8.0, 8.0));
  Position point;

  EXPECT_FALSE(iterator.isPastEnd());
  map.getPosition(*iterator, point);
  EXPECT_EQ(0, point.x());
  EXPECT_EQ(0, point.y());

  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(1, point.x());
  EXPECT_EQ(1, point.y());
  //
  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(2, point.x());
  EXPECT_EQ(2, point.y());
  //
  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(3, point.x());
  EXPECT_EQ(3, point.y());

  ++iterator;
  map.getPosition(*iterator, point);
  EXPECT_EQ(4, point.x());
  EXPECT_EQ(4, point.y());

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}
