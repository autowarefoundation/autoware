/*
 * GridMapDataIterator.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_core/GridMap.hpp"

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

TEST(GridMapIterator, Simple)
{
  GridMap map;
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(8, 5)
  map.add("layer", 0.0);
  GridMapIterator iterator(map);

  unsigned int i = 0;
  for (; !iterator.isPastEnd(); ++iterator, ++i) {
    map.at("layer", *iterator) = 1.0;
    EXPECT_FALSE(iterator.isPastEnd());
  }

  EXPECT_EQ(40, i);
  EXPECT_TRUE(iterator.isPastEnd());
  EXPECT_TRUE((map["layer"].array() == 1.0f).all());
}

TEST(GridMapIterator, LinearIndex)
{
  GridMap map;
  map.setGeometry(Length(8.1, 5.1), 1.0, Position(0.0, 0.0)); // bufferSize(8, 5)
  map.add("layer", 0.0);
  GridMapIterator iterator(map);

  auto& data = map["layer"];
  unsigned int i = 0;
  for (; !iterator.isPastEnd(); ++iterator, ++i) {
    data(iterator.getLinearIndex()) = 1.0;
    EXPECT_EQ(i, iterator.getLinearIndex());
    EXPECT_FALSE(iterator.isPastEnd());
  }

  EXPECT_EQ(40, i);
  EXPECT_TRUE(iterator.isPastEnd());
  EXPECT_TRUE((map["layer"].array() == 1.0f).all());
}
