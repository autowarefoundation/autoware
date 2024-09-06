// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_velocity_planner_common/collision_checker.hpp"

#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <random>

using autoware::motion_velocity_planner::CollisionChecker;
using autoware::universe_utils::Line2d;
using autoware::universe_utils::MultiLineString2d;
using autoware::universe_utils::MultiPoint2d;
using autoware::universe_utils::MultiPolygon2d;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

Point2d random_point()
{
  static std::random_device r;
  static std::default_random_engine e1(r());
  static std::uniform_real_distribution<double> uniform_dist(-100, 100);
  return {uniform_dist(e1), uniform_dist(e1)};
}

Line2d random_line()
{
  const auto point = random_point();
  const auto point2 = Point2d{point.x() + 1, point.y() + 1};
  const auto point3 = Point2d{point2.x() - 1, point2.y() + 1};
  const auto point4 = Point2d{point3.x() + 1, point3.y() + 1};
  return {point, point2, point3, point4};
}

Polygon2d random_polygon()
{
  Polygon2d polygon;
  const auto point = random_point();
  const auto point2 = Point2d{point.x() + 1, point.y() + 4};
  const auto point3 = Point2d{point.x() + 4, point.y() + 4};
  const auto point4 = Point2d{point.x() + 3, point.y() + 1};
  polygon.outer() = {point, point2, point3, point4, point};
  return polygon;
}

bool all_within(const MultiPoint2d & pts1, const MultiPoint2d & pts2)
{
  // results from the collision checker and the direct checks can have some small precision errors
  constexpr auto eps = 1e-2;
  for (const auto & p1 : pts1) {
    bool found = false;
    for (const auto & p2 : pts2) {
      if (boost::geometry::comparable_distance(p1, p2) < eps) {
        found = true;
        break;
      }
    }
    if (!found) return false;
  }
  return true;
}

TEST(TestCollisionChecker, Benchmark)
{
  constexpr auto nb_ego_footprints = 1000;
  constexpr auto nb_obstacles = 1000;
  MultiPolygon2d ego_footprints;
  ego_footprints.reserve(nb_ego_footprints);
  for (auto i = 0; i < nb_ego_footprints; ++i) {
    ego_footprints.push_back(random_polygon());
  }
  const auto cc_constr_start = std::chrono::system_clock::now();
  CollisionChecker collision_checker(ego_footprints);
  const auto cc_constr_end = std::chrono::system_clock::now();
  const auto cc_constr_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(cc_constr_end - cc_constr_start).count();
  std::printf(
    "Collision checker construction (with %d footprints): %ld ns\n", nb_ego_footprints,
    cc_constr_ns);
  MultiPolygon2d poly_obstacles;
  MultiPoint2d point_obstacles;
  MultiLineString2d line_obstacles;
  for (auto i = 0; i < nb_obstacles; ++i) {
    poly_obstacles.push_back(random_polygon());
    point_obstacles.push_back(random_point());
    line_obstacles.push_back(random_line());
  }
  const auto check_obstacles_one_by_one = [&](const auto & obstacles) {
    std::chrono::nanoseconds collision_checker_ns{};
    std::chrono::nanoseconds naive_ns{};
    for (const auto & obs : obstacles) {
      const auto cc_start = std::chrono::system_clock::now();
      const auto collisions = collision_checker.get_collisions(obs);
      MultiPoint2d cc_collision_points;
      for (const auto & c : collisions)
        cc_collision_points.insert(
          cc_collision_points.end(), c.collision_points.begin(), c.collision_points.end());
      const auto cc_end = std::chrono::system_clock::now();
      const auto naive_start = std::chrono::system_clock::now();
      MultiPoint2d naive_collision_points;
      for (const auto & ego_footprint : ego_footprints) {
        MultiPoint2d points;
        boost::geometry::intersection(ego_footprint, obs, points);
        naive_collision_points.insert(naive_collision_points.end(), points.begin(), points.end());
      }
      const auto naive_end = std::chrono::system_clock::now();
      const auto equal = all_within(cc_collision_points, naive_collision_points) &&
                         all_within(naive_collision_points, cc_collision_points);
      EXPECT_TRUE(equal);
      if (!equal) {
        std::cout << "cc: " << boost::geometry::wkt(cc_collision_points) << std::endl;
        std::cout << "naive: " << boost::geometry::wkt(naive_collision_points) << std::endl;
      }
      collision_checker_ns +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(cc_end - cc_start);
      naive_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(naive_end - naive_start);
    }
    std::printf("%20s%10ld ns\n", "collision checker : ", collision_checker_ns.count());
    std::printf("%20s%10ld ns\n", "naive : ", naive_ns.count());
  };
  const auto check_obstacles = [&](const auto & obstacles) {
    std::chrono::nanoseconds collision_checker_ns{};
    std::chrono::nanoseconds naive_ns{};
    const auto cc_start = std::chrono::system_clock::now();
    const auto collisions = collision_checker.get_collisions(obstacles);
    MultiPoint2d cc_collision_points;
    for (const auto & c : collisions)
      cc_collision_points.insert(
        cc_collision_points.end(), c.collision_points.begin(), c.collision_points.end());
    const auto cc_end = std::chrono::system_clock::now();
    const auto naive_start = std::chrono::system_clock::now();
    MultiPoint2d naive_collision_points;
    boost::geometry::intersection(ego_footprints, obstacles, naive_collision_points);
    const auto naive_end = std::chrono::system_clock::now();
    const auto equal = all_within(cc_collision_points, naive_collision_points) &&
                       all_within(naive_collision_points, cc_collision_points);
    EXPECT_TRUE(equal);
    if (!equal) {
      std::cout << "cc: " << boost::geometry::wkt(cc_collision_points) << std::endl;
      std::cout << "naive: " << boost::geometry::wkt(naive_collision_points) << std::endl;
    }
    collision_checker_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(cc_end - cc_start);
    naive_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(naive_end - naive_start);
    std::printf("%20s%10ld ns\n", "collision checker : ", collision_checker_ns.count());
    std::printf("%20s%10ld ns\n", "naive : ", naive_ns.count());
  };

  std::cout << "* check one by one\n";
  std::printf("%d Polygons:\n", nb_obstacles);
  check_obstacles_one_by_one(poly_obstacles);
  std::printf("%d Lines:\n", nb_obstacles);
  check_obstacles_one_by_one(line_obstacles);
  std::printf("%d Points:\n", nb_obstacles);
  check_obstacles_one_by_one(point_obstacles);
  std::cout << "* check all at once\n";
  std::printf("%d Polygons:\n", nb_obstacles);
  check_obstacles(poly_obstacles);
  std::printf("%d Lines:\n", nb_obstacles);
  check_obstacles(line_obstacles);
  std::printf("%d Points:\n", nb_obstacles);
  check_obstacles(point_obstacles);
}
