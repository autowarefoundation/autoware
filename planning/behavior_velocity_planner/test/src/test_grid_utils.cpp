// Copyright 2021 Tier IV, Inc.
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

#include "utils.hpp"

#include <scene_module/occlusion_spot/grid_utils.hpp>

#include <gtest/gtest.h>

#include <unordered_set>

struct indexHash
{
  std::size_t operator()(const grid_map::Index & i) const
  {
    return std::hash<int>()(i.x()) ^ std::hash<int>()(i.y());
  }
};
struct indexEq
{
  bool operator()(const grid_map::Index & i1, const grid_map::Index & i2) const
  {
    return i1.x() == i2.x() && i1.y() == i2.y();
  }
};

using test::grid_param;

TEST(isOcclusionSpotSquare, one_cell_occlusion_spot)
{
  using behavior_velocity_planner::grid_utils::isOcclusionSpotSquare;
  using behavior_velocity_planner::grid_utils::OcclusionSpotSquare;
  using behavior_velocity_planner::grid_utils::occlusion_cost_value::UNKNOWN;
  // Prepare an occupancy grid with a single occlusion_spot
  /*
      0 1 2 3
    0 ?   ? ?
    1   ? ?
    2   ? ? ?
    3 ?     ?
    */
  grid_map::GridMap grid = test::generateGrid(4, 4, 1.0);
  std::unordered_set<grid_map::Index, indexHash, indexEq> unknown_cells = {
    {0, 0}, {0, 3}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 2}, {3, 3}};
  for (grid_map::Index index : unknown_cells) {
    grid.at("layer", index) = UNKNOWN;
  }

  for (int i = 0; i < grid.getLength().x(); ++i) {
    for (int j = 0; j < grid.getLength().y(); ++j) {
      OcclusionSpotSquare occlusion_spot;
      bool found = isOcclusionSpotSquare(occlusion_spot, grid["layer"], {i, j}, 1, grid.getSize());
      if (unknown_cells.count({i, j})) {
        ASSERT_TRUE(found);
      } else {
        ASSERT_FALSE(found);
      }
    }
  }
}
TEST(isOcclusionSpotSquare, many_cells_occlusion_spot)
{
  using behavior_velocity_planner::grid_utils::isOcclusionSpotSquare;
  using behavior_velocity_planner::grid_utils::OcclusionSpotSquare;
  using behavior_velocity_planner::grid_utils::occlusion_cost_value::UNKNOWN;
  const double resolution = 1.0;
  const double size = 2 * resolution;
  /*
      0 1 2 3
    0 ?   ? ?
    1   ? ?
    2   ? ? ?
    3 ?     ?
    */
  grid_map::GridMap grid = test::generateGrid(4, 4, resolution);
  std::unordered_set<grid_map::Index, indexHash, indexEq> unknown_cells;
  std::unordered_set<grid_map::Index, indexHash, indexEq> occlusion_spot_cells;
  unknown_cells = {{0, 0}, {0, 3}, {1, 1}, {1, 2}, {2, 0}, {2, 1}, {2, 2}, {3, 0}, {3, 2}, {3, 3}};
  occlusion_spot_cells = {{2, 0}, {3, 0}, {1, 2}, {3, 3}};
  for (grid_map::Index index : unknown_cells) {
    grid.at("layer", index) = UNKNOWN;
  }

  for (int i = 0; i < grid.getLength().x(); ++i) {
    for (int j = 0; j < grid.getLength().y(); ++j) {
      OcclusionSpotSquare occlusion_spot;
      bool found =
        isOcclusionSpotSquare(occlusion_spot, grid["layer"], {i, j}, size, grid.getSize());
      if (occlusion_spot_cells.count({i, j})) {
        ASSERT_TRUE(found);
      } else {
        ASSERT_FALSE(found);
      }
    }
  }
  /*
      0 1 2 3
    0     ?
    1   ? ?
    2 ?   ? ?
    3
    */
  grid = test::generateGrid(4, 4, 1.0);
  unknown_cells = {{0, 2}, {1, 1}, {2, 0}, {2, 1}, {2, 2}, {3, 2}};
  for (grid_map::Index index : unknown_cells) {
    grid.at("layer", index) = UNKNOWN;
  }

  // No occlusion_spots
  for (int i = 0; i < grid.getLength().x(); ++i) {
    for (int j = 0; j < grid.getLength().y(); ++j) {
      OcclusionSpotSquare occlusion_spot;
      bool found =
        isOcclusionSpotSquare(occlusion_spot, grid["layer"], {i, j}, size, grid.getSize());
      ASSERT_FALSE(found);
    }
  }
  /*
      0 1 2 3
    0
    1   ? ?
    2   ? ?
    3
    */
  grid = test::generateGrid(4, 4, 1.0);
  unknown_cells = {{1, 1}, {1, 2}, {2, 1}, {2, 2}};
  for (grid_map::Index index : unknown_cells) {
    grid.at("layer", index) = UNKNOWN;
  }

  // Only one occlusion_spot square
  for (int i = 0; i < grid.getLength().x(); ++i) {
    for (int j = 0; j < grid.getLength().y(); ++j) {
      OcclusionSpotSquare occlusion_spot;
      bool found =
        isOcclusionSpotSquare(occlusion_spot, grid["layer"], {i, j}, size, grid.getSize());
      if (i == 1 && j == 2) {
        ASSERT_TRUE(found);
      } else {
        ASSERT_FALSE(found);
      }
    }
  }

  /*
      0 1 2 3
    0   ?
    1   ? ?
    2 ? ? ? ?
    3   ?
    */
  grid = test::generateGrid(4, 4, 1.0);
  unknown_cells = {{0, 2}, {1, 0}, {1, 1}, {1, 2}, {1, 3}, {2, 1}, {2, 2}, {3, 2}};
  for (grid_map::Index index : unknown_cells) {
    grid.at("layer", index) = UNKNOWN;
  }
  occlusion_spot_cells = {{1, 1}, {1, 2}, {2, 1}, {2, 2}};

  for (int i = 0; i < grid.getLength().x(); ++i) {
    for (int j = 0; j < grid.getLength().y(); ++j) {
      OcclusionSpotSquare occlusion_spot;
      bool found =
        isOcclusionSpotSquare(occlusion_spot, grid["layer"], {i, j}, size, grid.getSize());
      if (i == 1 && j == 2) {
        ASSERT_TRUE(found);
      } else {
        ASSERT_FALSE(found);
      }
    }
  }

  std::cout << "TooNoisyCase->No OcclusionSpot 2by2" << std::endl
            << "    0|1|2|3|4|                      " << std::endl
            << "  0  |?| | | |                      " << std::endl
            << "  1  |?|?|?| |                      " << std::endl
            << "  2 ?|?| |?| |                      " << std::endl
            << "  3  |?| | | |                      " << std::endl
            << "  4  |?| | | |                      " << std::endl;
  grid = test::generateGrid(5, 5, 1.0);
  unknown_cells = {{0, 2}, {1, 0}, {1, 1}, {1, 2}, {1, 3}, {2, 1}, {3, 1}, {3, 2}};
  for (grid_map::Index index : unknown_cells) {
    grid.at("layer", index) = UNKNOWN;
  }
  occlusion_spot_cells = {{1, 1}, {1, 2}, {2, 1}, {2, 2}};

  for (int i = 0; i < grid.getLength().x(); ++i) {
    for (int j = 0; j < grid.getLength().y(); ++j) {
      OcclusionSpotSquare occlusion_spot;
      bool found =
        isOcclusionSpotSquare(occlusion_spot, grid["layer"], {i, j}, size, grid.getSize());
      if (found) {
        std::cout << "i: " << i << " j: " << j << " change algorithm or update test" << std::endl;
      }
      ASSERT_FALSE(found);
    }
  }
}

TEST(buildSlices, test_buffer_offset)
{
  using behavior_velocity_planner::geometry::buildSlices;
  using behavior_velocity_planner::geometry::Slice;
  using behavior_velocity_planner::geometry::SliceRange;

  // Curving lanelet
  // lanelet::Lanelet l;
  // lanelet::Points3d line;
  boost::geometry::model::linestring<lanelet::BasicPoint2d> line;
  boost::geometry::model::multi_polygon<boost::geometry::model::polygon<lanelet::BasicPoint2d>>
    poly;
  line.emplace_back(0.0, 1.0);
  line.emplace_back(1.0, 4.0);
  line.emplace_back(2.0, 6.0);
  line.emplace_back(3.0, 7.0);
  line.emplace_back(4.0, 6.0);
  line.emplace_back(5.0, 4.0);
  line.emplace_back(6.0, 1.0);

  boost::geometry::strategy::buffer::distance_asymmetric<double> distance_strategy(0.0, 1.1);
  boost::geometry::strategy::buffer::end_flat end_strategy;
  boost::geometry::strategy::buffer::side_straight side_strategy;
  boost::geometry::strategy::buffer::join_miter join_strategy;
  boost::geometry::strategy::buffer::point_square point_strategy;
  boost::geometry::buffer(
    line, poly, distance_strategy, side_strategy, join_strategy, end_strategy, point_strategy);

  std::cout << boost::geometry::wkt(line) << "\n";
  std::cout << boost::geometry::wkt(poly[0]) << "\n";

  boost::geometry::model::multi_point<lanelet::BasicPoint2d> in1;
  for (const auto & p : line) {
    in1.push_back(p);
  }
  boost::geometry::model::multi_point<lanelet::BasicPoint2d> in2;
  for (const auto & p : poly[0].outer()) {
    in2.push_back(p);
  }
  boost::geometry::model::multi_point<lanelet::BasicPoint2d> output;
  boost::geometry::difference(in2, in1, output);

  std::cout << boost::geometry::wkt(output) << "\n";
}
