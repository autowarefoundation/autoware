/*
 * SlidingWindowIteratorTest.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich
 */

#include "grid_map_core/iterators/SlidingWindowIterator.hpp"
#include "grid_map_core/GridMap.hpp"

#include <cfloat>
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <vector>

using namespace std;
using namespace grid_map;

TEST(SlidingWindowIterator, WindowSize3Cutoff)
{
  GridMap map;
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(8, 5)
  map.add("layer");
  map["layer"].setRandom();

  SlidingWindowIterator iterator(map, "layer", SlidingWindowIterator::EdgeHandling::CROP, 3);
  EXPECT_EQ(iterator.getData().rows(), 2);
  EXPECT_EQ(iterator.getData().cols(), 2);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(0, 0, 2, 2)));

  ++iterator;
  EXPECT_EQ(iterator.getData().rows(), 3);
  EXPECT_EQ(iterator.getData().cols(), 2);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(0, 0, 3, 2)));

  ++iterator;
  EXPECT_EQ(iterator.getData().rows(), 3);
  EXPECT_EQ(iterator.getData().cols(), 2);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(1, 0, 3, 2)));

  for (; !iterator.isPastEnd(); ++iterator) {
    EXPECT_FALSE(iterator.isPastEnd());
    if ((*iterator == Index(3, 2)).all()) break;
  }

  EXPECT_EQ(iterator.getData().rows(), 3);
  EXPECT_EQ(iterator.getData().cols(), 3);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(2, 1, 3, 3)));

  for (; !iterator.isPastEnd(); ++iterator) {
    EXPECT_FALSE(iterator.isPastEnd());
    if ((*iterator == Index(7, 4)).all()) break;
  }

  EXPECT_EQ(iterator.getData().rows(), 2);
  EXPECT_EQ(iterator.getData().cols(), 2);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(6, 3, 2, 2)));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(SlidingWindowIterator, WindowSize5)
{
  GridMap map;
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(8, 5)
  map.add("layer");
  map["layer"].setRandom();

  SlidingWindowIterator iterator(map, "layer", SlidingWindowIterator::EdgeHandling::CROP, 5);
  EXPECT_EQ(iterator.getData().rows(), 3);
  EXPECT_EQ(iterator.getData().cols(), 3);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(0, 0, 3, 3)));

  ++iterator;
  EXPECT_EQ(iterator.getData().rows(), 4);
  EXPECT_EQ(iterator.getData().cols(), 3);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(0, 0, 4, 3)));

  ++iterator;
  EXPECT_EQ(iterator.getData().rows(), 5);
  EXPECT_EQ(iterator.getData().cols(), 3);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(0, 0, 5, 3)));

  for (; !iterator.isPastEnd(); ++iterator) {
    EXPECT_FALSE(iterator.isPastEnd());
    if ((*iterator == Index(3, 2)).all()) break;
  }

  EXPECT_EQ(iterator.getData().rows(), 5);
  EXPECT_EQ(iterator.getData().cols(), 5);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(1, 0, 5, 5)));

  for (; !iterator.isPastEnd(); ++iterator) {
    EXPECT_FALSE(iterator.isPastEnd());
    if ((*iterator == Index(7, 4)).all()) break;
  }

  EXPECT_EQ(iterator.getData().rows(), 3);
  EXPECT_EQ(iterator.getData().cols(), 3);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(5, 2, 3, 3)));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(SlidingWindowIterator, WindowSize3Inside)
{
  GridMap map;
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(8, 5)
  map.add("layer");
  map["layer"].setRandom();

  SlidingWindowIterator iterator(map, "layer", SlidingWindowIterator::EdgeHandling::INSIDE, 3);
  EXPECT_EQ(iterator.getData().rows(), 3);
  EXPECT_EQ(iterator.getData().cols(), 3);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(0, 0, 3, 3)));

  for (; !iterator.isPastEnd(); ++iterator) {
    EXPECT_FALSE(iterator.isPastEnd());
    if ((*iterator == Index(3, 2)).all()) break;
  }

  EXPECT_EQ(iterator.getData().rows(), 3);
  EXPECT_EQ(iterator.getData().cols(), 3);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(2, 1, 3, 3)));

  for (; !iterator.isPastEnd(); ++iterator) {
    EXPECT_FALSE(iterator.isPastEnd());
    if ((*iterator == Index(6, 3)).all()) break;
  }

  EXPECT_EQ(iterator.getData().rows(), 3);
  EXPECT_EQ(iterator.getData().cols(), 3);
  EXPECT_TRUE(iterator.getData().isApprox(map["layer"].block(5, 2, 3, 3)));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}
