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

#include "../src/obstacles.hpp"
#include "../src/types.hpp"

#include <chrono>
#include <random>

using autoware::motion_velocity_planner::obstacle_velocity_limiter::CollisionChecker;
using autoware::motion_velocity_planner::obstacle_velocity_limiter::linestring_t;
using autoware::motion_velocity_planner::obstacle_velocity_limiter::Obstacles;
using autoware::motion_velocity_planner::obstacle_velocity_limiter::point_t;
using autoware::motion_velocity_planner::obstacle_velocity_limiter::polygon_t;

point_t random_point()
{
  static std::random_device r;
  static std::default_random_engine e1(r());
  static std::uniform_real_distribution<double> uniform_dist(-100, 100);
  return {uniform_dist(e1), uniform_dist(e1)};
}

linestring_t random_line()
{
  const auto point = random_point();
  const auto point2 = point_t{point.x() + 1, point.y() + 1};
  const auto point3 = point_t{point2.x() - 1, point2.y() + 1};
  const auto point4 = point_t{point3.x() + 1, point3.y() + 1};
  return {point, point2, point3, point4};
}

polygon_t random_polygon()
{
  polygon_t polygon;
  const auto point = random_point();
  const auto point2 = point_t{point.x() + 1, point.y() + 4};
  const auto point3 = point_t{point.x() + 4, point.y() + 4};
  const auto point4 = point_t{point.x() + 3, point.y() + 1};
  polygon.outer() = {point, point2, point3, point4, point};
  return polygon;
}

int main()
{
  Obstacles obstacles;
  std::vector<polygon_t> polygons;
  polygons.reserve(100);
  for (auto i = 0; i < 100; ++i) {
    polygons.push_back(random_polygon());
  }
  for (auto nb_lines = 1lu; nb_lines < 1000lu; nb_lines += 10) {
    obstacles.lines.push_back(random_line());
    obstacles.points.clear();
    for (auto nb_points = 1lu; nb_points < 1000lu; nb_points += 10) {
      obstacles.points.push_back(random_point());
      const auto rtt_constr_start = std::chrono::system_clock::now();
      CollisionChecker rtree_collision_checker(obstacles, 0, 0);
      const auto rtt_constr_end = std::chrono::system_clock::now();
      const auto naive_constr_start = std::chrono::system_clock::now();
      CollisionChecker naive_collision_checker(obstacles, nb_points + 1, nb_lines * 100);
      const auto naive_constr_end = std::chrono::system_clock::now();
      const auto rtt_check_start = std::chrono::system_clock::now();
      for (const auto & polygon : polygons)
        // cppcheck-suppress unreadVariable
        const auto rtree_result = rtree_collision_checker.intersections(polygon);
      const auto rtt_check_end = std::chrono::system_clock::now();
      const auto naive_check_start = std::chrono::system_clock::now();
      for (const auto & polygon : polygons)
        // cppcheck-suppress unreadVariable
        const auto naive_result = naive_collision_checker.intersections(polygon);
      const auto naive_check_end = std::chrono::system_clock::now();
      const auto rtt_constr_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(rtt_constr_end - rtt_constr_start);
      const auto naive_constr_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(naive_constr_end - naive_constr_start);
      const auto rtt_check_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(rtt_check_end - rtt_check_start);
      const auto naive_check_time =
        std::chrono::duration_cast<std::chrono::nanoseconds>(naive_check_end - naive_check_start);
      std::printf(
        "%lu, %lu, %ld, %ld, %ld, %ld\n", nb_lines, nb_points, rtt_constr_time.count(),
        rtt_check_time.count(), naive_constr_time.count(), naive_check_time.count());
    }
  }
  return 0;
}
