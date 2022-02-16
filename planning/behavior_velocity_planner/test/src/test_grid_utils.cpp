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

TEST(isOcclusionSpotSquare, occlusion_spot_cell)
{
  using behavior_velocity_planner::grid_utils::isOcclusionSpotSquare;
  using behavior_velocity_planner::grid_utils::OcclusionSpotSquare;
  using behavior_velocity_planner::grid_utils::occlusion_cost_value::UNKNOWN;
  // Prepare an occupancy grid with a single occlusion_spot
  const int min_occlusion_spot_size = 2;

  // case simple
  {
    /*
        0 1 2 3 4
      0
      1   ?   ?
      2   ? x ?
      3   ? ?
      4
      */
    grid_map::GridMap grid = test::generateGrid(5, 5, 1.0);
    std::vector<grid_map::Index> unknown_cells = {{1, 1}, {1, 2}, {1, 3}, {2, 2},
                                                  {2, 3}, {3, 1}, {3, 2}};
    for (grid_map::Index index : unknown_cells) {
      grid.at("layer", index) = UNKNOWN;
    }

    // occlusion spot (2,2)
    for (int i = 0; i < grid.getLength().x(); ++i) {
      for (int j = 0; j < grid.getLength().y(); ++j) {
        OcclusionSpotSquare occlusion_spot;
        bool found = isOcclusionSpotSquare(
          occlusion_spot, grid["layer"], {i, j}, min_occlusion_spot_size, grid.getSize());
        if (i == 2 && j == 2) {
          EXPECT_TRUE(found);
        } else {
          EXPECT_FALSE(found);
        }
      }
    }
  }
  // case noisy
  {
    std::cout << "TooNoisyCase->No OcclusionSpot 2by2" << std::endl
              << "    0|1|2|3|4|                      " << std::endl
              << "  0  |?| | | |                      " << std::endl
              << "  1  |?| | | |                      " << std::endl
              << "  2  |?|?|?| |                      " << std::endl;
    grid_map::GridMap grid = test::generateGrid(5, 3, 1.0);
    std::vector<grid_map::Index> unknown_cells = {{1, 0}, {1, 1}, {1, 2}, {2, 2}, {3, 2}};
    for (grid_map::Index index : unknown_cells) {
      grid.at("layer", index) = UNKNOWN;
    }
    for (int i = 0; i < grid.getLength().x(); ++i) {
      for (int j = 0; j < grid.getLength().y(); ++j) {
        OcclusionSpotSquare occlusion_spot;
        bool found = isOcclusionSpotSquare(
          occlusion_spot, grid["layer"], {i, j}, min_occlusion_spot_size, grid.getSize());
        if (found) {
          std::cout << "i: " << i << " j: " << j << " change algorithm or update test" << std::endl;
        }
        ASSERT_FALSE(found);
      }
    }
  }
}
