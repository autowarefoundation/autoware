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
#include <tier4_autoware_utils/system/stop_watch.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <gtest/gtest.h>

#include <cmath>
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

using behavior_velocity_planner::LineString2d;
using behavior_velocity_planner::Point2d;
using behavior_velocity_planner::Polygon2d;
using behavior_velocity_planner::grid_utils::occlusion_cost_value::OCCUPIED;
using behavior_velocity_planner::grid_utils::occlusion_cost_value::UNKNOWN;

Polygon2d pointsToPoly(const Point2d p0, const Point2d p1, const double radius)
{
  LineString2d line = {p0, p1};
  const double angle = atan2(p0.y() - p1.y(), p0.x() - p1.x());
  const double r = radius;
  Polygon2d line_poly;
  // add polygon counter clockwise
  line_poly.outer().emplace_back(p0.x() + r * sin(angle), p0.y() - r * cos(angle));
  line_poly.outer().emplace_back(p1.x() + r * sin(angle), p1.y() - r * cos(angle));
  line_poly.outer().emplace_back(p1.x() - r * sin(angle), p1.y() + r * cos(angle));
  line_poly.outer().emplace_back(p0.x() - r * sin(angle), p0.y() + r * cos(angle));
  // std::cout << boost::geometry::wkt(line_poly) << std::endl;
  // std::cout << boost::geometry::wkt(line) << std::endl;
  return line_poly;
}

TEST(compareTime, polygon_vs_line_iterator)
{
  // Prepare an occupancy grid with a single occlusion_spot

  /*
      0 1 2 ....200
    0
    1   ? ? ?    ?
    2   ? ? ?    .
    .   ? ? ?    ?
    200 ? ? ? ...?
    */
  const size_t cell_size = 200;
  grid_map::GridMap grid = test::generateGrid(cell_size, cell_size, 0.5);
  std::vector<grid_map::Index> unknown_cells(cell_size);
  for (int i = 0; i < grid.getLength().x(); ++i) {
    for (int j = 0; j < grid.getLength().y(); ++j) {
      const grid_map::Index index = {i, j};
      grid.at("layer", index) = UNKNOWN;
    }
  }
  const grid_map::Matrix & grid_data = grid["layer"];
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("processing_time");
  size_t count = 0;
  [[maybe_unused]] double time = 0;
  for (size_t i = 0; i < 10; i++) {
    const double length = static_cast<double>(cell_size * i) * 0.005 * std::sqrt(2);
    // std::cout << "length of polygon: " << length << std::endl;
    // polygon iterator
    {
      const Point2d p0(0, 0);
      const Point2d p1(length, length);
      const Polygon2d polygon = pointsToPoly(p0, p1, 0.5);
      grid_map::Polygon grid_polygon;
      try {
        for (const auto & point : polygon.outer()) {
          grid_polygon.addVertex({point.x(), point.y()});
        }
        for (grid_map::PolygonIterator iterator(grid, grid_polygon); !iterator.isPastEnd();
             ++iterator) {
          const grid_map::Index & index = *iterator;
          if (grid_data(index.x(), index.y()) == OCCUPIED) {
            std::cout << "occupied" << std::endl;
            // occupied
          } else {
            count++;
            // continue
          }
        }
      } catch (const std::invalid_argument & e) {
        std::cerr << e.what() << std::endl;
      }
    }
    time = stop_watch.toc("processing_time", true);
    // if (i < 4) std::cout << "polygon iterator [ms]: " << time << " :num: " << count << std::endl;
    count = 0;
    // line iterator
    {
      const grid_map::Position p0(0, 0);
      const grid_map::Position p1(length, length);
      try {
        for (grid_map::LineIterator iterator(grid, p0, p1); !iterator.isPastEnd(); ++iterator) {
          const grid_map::Index & index = *iterator;
          if (grid_data(index.x(), index.y()) == OCCUPIED) {
            std::cout << "occupied" << std::endl;
            // occupied
          } else {
            for (int i = -1; i <= 1; i++) {
              for (int j = -1; j <= 1; j++) {
                if (grid_data(index.x() + i, index.y() + j) == OCCUPIED) {
                  // for compare
                }
                count++;
              }
            }
            // continue
          }
        }
      } catch (const std::invalid_argument & e) {
        std::cerr << e.what() << std::endl;
      }
    }
    time = stop_watch.toc("processing_time", true);
    // if (i < 4) std::cout << "line iterator [ms]: " << time << " :num: " << count << std::endl;
    count = 0;
  }
}
