// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_velocity_limiter/occupancy_grid_utils.hpp"
#include "obstacle_velocity_limiter/types.hpp"

#include <boost/geometry/algorithms/correct.hpp>

#include <gtest/gtest.h>

TEST(TestOccupancyGridUtils, extractObstacleLines)
{
  using obstacle_velocity_limiter::multipolygon_t;
  using obstacle_velocity_limiter::polygon_t;
  constexpr int8_t occupied_thr = 10;
  nav_msgs::msg::OccupancyGrid occupancy_grid;
  occupancy_grid.info.height = 5;
  occupancy_grid.info.width = 5;
  occupancy_grid.data =
    std::vector<signed char>(occupancy_grid.info.height * occupancy_grid.info.width);
  occupancy_grid.info.resolution = 1.0;

  polygon_t full_mask;
  full_mask.outer() = {{0.0, 0.0}, {0.0, 5.0}, {5.0, 5.0}, {5.0, 0.0}, {0.0, 0.0}};

  constexpr auto extractObstacles = [](
                                      const nav_msgs::msg::OccupancyGrid & occupancy_grid,
                                      const multipolygon_t & negative_masks,
                                      const polygon_t & positive_mask, const double thr) {
    obstacle_velocity_limiter::ObstacleMasks masks;
    masks.negative_masks = negative_masks;
    masks.positive_mask = positive_mask;
    auto grid_map = obstacle_velocity_limiter::convertToGridMap(occupancy_grid);
    obstacle_velocity_limiter::threshold(grid_map, thr);
    obstacle_velocity_limiter::maskPolygons(grid_map, masks);
    return obstacle_velocity_limiter::extractObstacles(grid_map, occupancy_grid);
  };
  auto obstacles = extractObstacles(occupancy_grid, {}, full_mask, occupied_thr);
  EXPECT_TRUE(obstacles.empty());

  occupancy_grid.data.at(5) = 10;
  obstacles = extractObstacles(occupancy_grid, {}, full_mask, occupied_thr);
  EXPECT_EQ(obstacles.size(), 1ul);

  for (auto i = 1; i < 4; ++i)
    for (auto j = 1; j < 4; ++j) occupancy_grid.data[j + i * occupancy_grid.info.width] = 10;
  obstacles = extractObstacles(occupancy_grid, {}, full_mask, occupied_thr);
  EXPECT_EQ(obstacles.size(), 1ul);

  obstacles = extractObstacles(occupancy_grid, {full_mask}, full_mask, occupied_thr);
  EXPECT_EQ(obstacles.size(), 0ul);
}
