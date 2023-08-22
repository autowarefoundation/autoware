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

#include "obstacle_velocity_limiter/obstacles.hpp"
#include "obstacle_velocity_limiter/types.hpp"

#include <boost/geometry/io/wkt/write.hpp>

#include <gtest/gtest.h>

TEST(TestObstacles, ObstacleTreePoints)
{
  /*
  using obstacle_velocity_limiter::point_t;
  const std::vector<point_t> points = {point_t(0, 0), point_t(2, 2), point_t(10, -5)};
  obstacle_velocity_limiter::ObstacleTree tree(points);

  obstacle_velocity_limiter::polygon_t query;
  query.outer() = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
  auto result = tree.intersections(query);
  ASSERT_EQ(result.size(), 1lu);
  EXPECT_EQ(result[0].x(), 0);
  EXPECT_EQ(result[0].y(), 0);

  query.outer() = {{-1, -1}, {-1, 1}, {10, 10}, {10, -1}, {-1, -1}};
  result = tree.intersections(query);
  ASSERT_EQ(result.size(), 2lu);
  EXPECT_EQ(result[0].x(), 0);
  EXPECT_EQ(result[0].y(), 0);
  EXPECT_EQ(result[1].x(), 2);
  EXPECT_EQ(result[1].y(), 2);
  */
}

TEST(TestObstacles, ObstacleTreeLines)
{
  /*
  using obstacle_velocity_limiter::point_t;
  using obstacle_velocity_limiter::linestring_t;
  const obstacle_velocity_limiter::multi_linestring_t lines = {
      {point_t{-0.5, -0.5}, point_t{0.5,0.5}},
      {point_t{0, 0}, point_t{1,1}},
      {point_t(2, 2), point_t(-2, 2)},
      {point_t{-3,-3}, point_t{-1,-1}, point_t{5, -5}}
  };
  obstacle_velocity_limiter::ObstacleTree tree(lines);

  obstacle_velocity_limiter::polygon_t query;
  query.outer() = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
  auto result = tree.intersections(query);
  EXPECT_EQ(result.size(), 4lu);
  for(const auto & point : result)
      std::cout << boost::geometry::wkt(point) << std::endl;

  query.outer() = {{-2, 10}, {-1, 10}, {-1, -10}, {-2, -10}, {-2, 10}};
  result = tree.intersections(query);
  EXPECT_EQ(result.size(), 4lu);
  for(const auto & point : result)
      std::cout << boost::geometry::wkt(point) << std::endl;

  query.outer() = {{0.1, -0.1}, {1, -0.1}, {1, -1}, {0.1, -1}, {0.1, -0.1}};
  result = tree.intersections(query);
  for(const auto & point : result)
      std::cout << boost::geometry::wkt(point) << std::endl;
  */
}
