// Copyright 2022 Tier IV, Inc.
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

#include "grid_map_utils/polygon_iterator.hpp"

#include <grid_map_core/iterators/PolygonIterator.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

// gtest
#include <gtest/gtest.h>

// Vector
#include <random>
#include <string>
#include <vector>

using grid_map::GridMap;
using grid_map::Index;
using grid_map::Length;
using grid_map::Polygon;
using grid_map::Position;

// Copied from grid_map::PolygonIterator
TEST(PolygonIterator, FullCover)
{
  std::vector<std::string> types;
  types.emplace_back("type");
  GridMap map(types);
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-100.0, 100.0));
  polygon.addVertex(Position(100.0, 100.0));
  polygon.addVertex(Position(100.0, -100.0));
  polygon.addVertex(Position(-100.0, -100.0));

  grid_map_utils::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 37; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

// Copied from grid_map::PolygonIterator
TEST(PolygonIterator, Outside)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(99.0, 101.0));
  polygon.addVertex(Position(101.0, 101.0));
  polygon.addVertex(Position(101.0, 99.0));
  polygon.addVertex(Position(99.0, 99.0));

  grid_map_utils::PolygonIterator iterator(map, polygon);

  EXPECT_TRUE(iterator.isPastEnd());
}

// Copied from grid_map::PolygonIterator
TEST(PolygonIterator, Square)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-1.0, 1.5));
  polygon.addVertex(Position(1.0, 1.5));
  polygon.addVertex(Position(1.0, -1.5));
  polygon.addVertex(Position(-1.0, -1.5));

  grid_map_utils::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

// Copied from grid_map::PolygonIterator
TEST(PolygonIterator, TopLeftTriangle)
{
  GridMap map({"types"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  Polygon polygon;
  polygon.addVertex(Position(-40.1, 20.6));
  polygon.addVertex(Position(40.1, 20.4));
  polygon.addVertex(Position(-40.1, -20.6));

  grid_map_utils::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));
  EXPECT_EQ(0, (*iterator)(1));
}

// Copied from grid_map::PolygonIterator
TEST(PolygonIterator, MoveMap)
{
  GridMap map({"layer"});
  map.setGeometry(Length(8.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)
  map.move(Position(2.0, 0.0));

  Polygon polygon;
  polygon.addVertex(Position(6.1, 1.6));
  polygon.addVertex(Position(0.9, 1.6));
  polygon.addVertex(Position(0.9, -1.6));
  polygon.addVertex(Position(6.1, -1.6));
  grid_map_utils::PolygonIterator iterator(map, polygon);

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  for (int i = 0; i < 4; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(0, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  for (int i = 0; i < 8; ++i) ++iterator;

  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

// This test shows a difference when an edge passes exactly through the center of a cell
TEST(PolygonIterator, Difference)
{
  GridMap map({"layer"});
  map.setGeometry(Length(5.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  // Triangle where the hypothenus is an exact diagonal of the map: difference.
  Polygon polygon;
  polygon.addVertex(Position(2.5, 2.5));
  polygon.addVertex(Position(-2.5, 2.5));
  polygon.addVertex(Position(-2.5, -2.5));
  grid_map_utils::PolygonIterator iterator(map, polygon);
  grid_map::PolygonIterator gm_iterator(map, polygon);
  bool diff = false;
  while (!iterator.isPastEnd() && !gm_iterator.isPastEnd()) {
    if ((*gm_iterator)(0) != (*iterator)(0) || (*gm_iterator)(1) != (*iterator)(1)) diff = true;
    ++iterator;
    ++gm_iterator;
  }
  if (iterator.isPastEnd() != gm_iterator.isPastEnd()) {
    diff = true;
  }
  EXPECT_TRUE(diff);

  // Triangle where the hypothenus does not cross any cell center: no difference.
  polygon.removeVertices();
  polygon.addVertex(Position(2.5, 2.1));
  polygon.addVertex(Position(-2.5, 2.5));
  polygon.addVertex(Position(-2.5, -2.9));
  iterator = grid_map_utils::PolygonIterator(map, polygon);
  gm_iterator = grid_map::PolygonIterator(map, polygon);
  diff = false;
  while (!iterator.isPastEnd() && !gm_iterator.isPastEnd()) {
    if ((*gm_iterator)(0) != (*iterator)(0) || (*gm_iterator)(1) != (*iterator)(1)) diff = true;
    ++iterator;
    ++gm_iterator;
  }
  if (iterator.isPastEnd() != gm_iterator.isPastEnd()) {
    diff = true;
  }
  EXPECT_FALSE(diff);
}

TEST(PolygonIterator, SelfCrossingPolygon)
{
  GridMap map({"layer"});
  map.setGeometry(Length(5.0, 5.0), 1.0, Position(0.0, 0.0));  // bufferSize(8, 5)

  // Hour-glass shape
  Polygon polygon;
  polygon.addVertex(Position(2.5, 2.9));
  polygon.addVertex(Position(2.5, -2.9));
  polygon.addVertex(Position(-2.5, 2.5));
  polygon.addVertex(Position(-2.5, -2.5));
  grid_map_utils::PolygonIterator iterator(map, polygon);
  grid_map::PolygonIterator gm_iterator(map, polygon);

  const std::vector<Index> expected_indexes = {
    Index(0, 0), Index(0, 1), Index(0, 2), Index(0, 3), Index(0, 4), Index(1, 1), Index(1, 2),
    Index(1, 3), Index(2, 2), Index(3, 2), Index(4, 1), Index(4, 2), Index(4, 3)};
  bool diff = false;
  size_t i = 0;
  while (!iterator.isPastEnd() && !gm_iterator.isPastEnd()) {
    if ((*gm_iterator)(0) != (*iterator)(0) || (*gm_iterator)(1) != (*iterator)(1)) diff = true;
    ASSERT_TRUE(i < expected_indexes.size());
    EXPECT_EQ((*iterator)(0), expected_indexes[i](0));
    EXPECT_EQ((*iterator)(1), expected_indexes[i](1));
    ++i;
    ++iterator;
    ++gm_iterator;
  }
  if (iterator.isPastEnd() != gm_iterator.isPastEnd()) {
    diff = true;
  }
  EXPECT_FALSE(diff);
}
