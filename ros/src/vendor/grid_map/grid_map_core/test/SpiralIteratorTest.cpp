/*
 * SpiralIteratorTest.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: Benjamin Scholz
 *	 Institute: University of Hamburg, TAMS
 */

#include "grid_map_core/iterators/SpiralIterator.hpp"
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
using namespace Eigen;
using namespace grid_map;

TEST(SpiralIterator, CenterOutOfMap)
{
  GridMap map( { "types" });
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));
  Position center(8.0, 0.0);
  double radius = 5.0;

  SpiralIterator iterator(map, center, radius);

  Position iterator_position;
  map.getPosition(*iterator, iterator_position);

  EXPECT_TRUE(map.isInside(iterator_position));
}
