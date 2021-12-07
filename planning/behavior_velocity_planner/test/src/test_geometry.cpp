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

#include <scene_module/occlusion_spot/geometry.hpp>

#include <gtest/gtest.h>

#include <vector>

TEST(lerp, interpolation)
{
  using behavior_velocity_planner::geometry::lerp;
  using V = Eigen::Vector2d;
  // 1D
  EXPECT_EQ(lerp(0.0, 1.0, 0.5), 0.5);
  EXPECT_EQ(lerp(0.0, 1.0, 0.0), 0.0);
  EXPECT_EQ(lerp(0.0, 1.0, 1.0), 1.0);
  EXPECT_EQ(lerp(0.0, 1.0, 4.5), 4.5);
  EXPECT_EQ(lerp(1.0, 2.0, 0.5), 1.5);
  EXPECT_EQ(lerp(1.0, 2.0, 0.75), 1.75);
  EXPECT_EQ(lerp(0.0, -2.0, 0.75), -1.5);
  EXPECT_EQ(lerp(-10.0, -5.0, 0.5), -7.5);
  // 2D
  auto expect_eq = [](V v1, V v2) {
    EXPECT_EQ(v1.x(), v2.x());
    EXPECT_EQ(v1.y(), v2.y());
  };
  expect_eq(lerp(V(0.0, 0.0), V(0.0, 1.0), 0.5), V(0.0, 0.5));
  expect_eq(lerp(V(0.0, 1.0), V(0.0, 2.0), 0.5), V(0.0, 1.5));
  expect_eq(lerp(V(0.0, 0.0), V(2.0, 2.0), 0.5), V(1.0, 1.0));
  expect_eq(lerp(V(-100.0, 10.0), V(0.0, 0.0), 0.1), V(-90.0, 9.0));
}

TEST(buildInterpolatedPolygon, straight_polygon)
{
  using behavior_velocity_planner::geometry::buildInterpolatedPolygon;
  using V = Eigen::Vector2d;
  auto expect_eq = [](V v1, V v2) {
    EXPECT_EQ(v1.x(), v2.x());
    EXPECT_EQ(v1.y(), v2.y());
  };

  lanelet::BasicLineString2d from;
  lanelet::BasicLineString2d to;
  double from_length;
  double to_length;
  double from_ratio_dist;
  double to_ratio_dist;
  lanelet::BasicPolygon2d polygon;

  from = {{0.0, 0.0}, {0.0, 10.0}};
  to = {{10.0, 0.0}, {10.0, 10.0}};
  from_length = 0.0;
  to_length = 10.0;
  from_ratio_dist = 0.0;
  to_ratio_dist = 1.0;
  buildInterpolatedPolygon(
    polygon, from, to, from_length, to_length, from_ratio_dist, to_ratio_dist);
  ASSERT_EQ(polygon.size(), static_cast<size_t>(4));
  expect_eq(polygon[0], from[0]);
  expect_eq(polygon[1], from[1]);
  expect_eq(polygon[2], to[1]);
  expect_eq(polygon[3], to[0]);

  from_ratio_dist = 0.0;
  to_ratio_dist = 0.5;
  buildInterpolatedPolygon(
    polygon, from, to, from_length, to_length, from_ratio_dist, to_ratio_dist);
  ASSERT_EQ(polygon.size(), static_cast<size_t>(4));
  expect_eq(polygon[0], V(0.0, 0.0));
  expect_eq(polygon[1], V(0.0, 10.0));
  expect_eq(polygon[2], V(5.0, 10.0));
  expect_eq(polygon[3], V(5.0, 0.0));

  from_ratio_dist = 0.25;
  to_ratio_dist = 0.5;
  buildInterpolatedPolygon(
    polygon, from, to, from_length, to_length, from_ratio_dist, to_ratio_dist);
  ASSERT_EQ(polygon.size(), static_cast<size_t>(4));
  expect_eq(polygon[0], V(2.5, 0.0));
  expect_eq(polygon[1], V(2.5, 10.0));
  expect_eq(polygon[2], V(5.0, 10.0));
  expect_eq(polygon[3], V(5.0, 0.0));

  from_length = 1.5;
  to_length = 7.5;
  buildInterpolatedPolygon(
    polygon, from, to, from_length, to_length, from_ratio_dist, to_ratio_dist);
  ASSERT_EQ(polygon.size(), static_cast<size_t>(4));
  expect_eq(polygon[0], V(2.5, 1.5));
  expect_eq(polygon[1], V(2.5, 7.5));
  expect_eq(polygon[2], V(5.0, 7.5));
  expect_eq(polygon[3], V(5.0, 1.5));
}

TEST(buildSlices, one_full_slice)
{
  using behavior_velocity_planner::geometry::buildSlices;
  using behavior_velocity_planner::geometry::Slice;
  using behavior_velocity_planner::geometry::SliceRange;

  /* Simple vertical lanelet
     3x3 slice on the left
        0 1 2 3 4 5 6
      0 | |------
      1 | |     |
      2 | |SLICE|
      3 | |-----|
      4 | |
      5 | |
      */
  const int nb_points = 6;
  lanelet::ConstLanelet traj_lanelet = test::verticalLanelet({0.0, 0.0}, 1.0, 5.0, nb_points);
  SliceRange range;
  range.min_length = 0.0;
  range.max_length = 3.0;
  range.min_distance = 0.0;
  range.max_distance = -3.0;
  const double slice_length = 3.0;
  const double slice_width = 3.0;
  std::vector<Slice> slices;
  buildSlices(slices, traj_lanelet, range, slice_length, slice_width);
  ASSERT_EQ(slices.size(), static_cast<size_t>(1));
  EXPECT_EQ(slices[0].range.min_length, 0.0);
  EXPECT_EQ(slices[0].range.min_distance, 0.0);
  EXPECT_EQ(slices[0].range.max_length, 3.0);
  EXPECT_EQ(slices[0].range.max_distance, -3.0);
}

TEST(buildSlices, 3x3square_slice)
{
  using behavior_velocity_planner::geometry::buildSlices;
  using behavior_velocity_planner::geometry::Slice;
  using behavior_velocity_planner::geometry::SliceRange;

  /* Simple vertical lanelet
     3x3 slice on the left
        0 1 2 3 4 5 6
      0 | |------
      1 | |     |
      2 | |SLICE|
      3 | |-----|
      4 | |
      5 | |
      */
  const int nb_points = 6;
  lanelet::ConstLanelet traj_lanelet = test::verticalLanelet({0.0, 0.0}, 1.0, 5.0, nb_points);
  SliceRange range;
  range.min_length = 0.0;
  range.max_length = 3.0;
  range.min_distance = 0.0;
  range.max_distance = -3.0;
  const double slice_length = 1.0;
  const double slice_width = 1.0;
  {
    std::vector<Slice> slices;
    buildSlices(slices, traj_lanelet, range, slice_length, slice_width);
    SliceRange ref;
    auto equal_range = [&ref](const Slice & s) {
      return s.range.min_distance == ref.min_distance && s.range.max_distance == ref.max_distance &&
             s.range.min_length == ref.min_length && s.range.max_length == ref.max_length;
    };
    ASSERT_EQ(slices.size(), static_cast<size_t>(9));
    for (double l = range.min_length; l < range.max_length; l += slice_length) {
      for (double d = range.min_distance; d > range.max_distance; d -= slice_width) {
        ref.min_length = l;
        ref.max_length = l + slice_length;
        ref.min_distance = d;
        ref.max_distance = d - slice_width;
        EXPECT_NE(std::find_if(slices.begin(), slices.end(), equal_range), slices.end());
      }
    }
  }

  // change to the left side (positive distance)
  range.max_distance = 3.0;
  {
    std::vector<Slice> slices;
    buildSlices(slices, traj_lanelet, range, slice_length, slice_width);
    SliceRange ref;
    auto equal_range = [&ref](const Slice & s) {
      return s.range.min_distance == ref.min_distance && s.range.max_distance == ref.max_distance &&
             s.range.min_length == ref.min_length && s.range.max_length == ref.max_length;
    };
    ASSERT_EQ(slices.size(), static_cast<size_t>(9));
    for (double l = range.min_length; l < range.max_length; l += slice_length) {
      for (double d = range.min_distance; d < range.max_distance; d += slice_width) {
        ref.min_length = l;
        ref.max_length = l + slice_length;
        ref.min_distance = d;
        ref.max_distance = d + slice_width;
        EXPECT_NE(std::find_if(slices.begin(), slices.end(), equal_range), slices.end());
      }
    }
  }
}
