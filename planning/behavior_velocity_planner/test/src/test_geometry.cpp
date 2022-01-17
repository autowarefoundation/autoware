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
  range.min_distance = 0.0;
  range.max_distance = -3.0;
  const double slice_length = 3.0;
  const double slice_width = 1.5;
  std::vector<Slice> slices;
  buildSlices(slices, traj_lanelet, range, slice_length, slice_width, 0.5);
  ASSERT_EQ(slices.size(), static_cast<size_t>(4));
  EXPECT_EQ(slices[0].range.min_length, 0.0);
  EXPECT_EQ(slices[0].range.min_distance, 0.0);
  EXPECT_EQ(slices[0].range.max_length, 3.0);
  EXPECT_EQ(slices[0].range.max_distance, -1.5);
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
      */
  const int nb_points = 4;
  const double len = 3.0;
  lanelet::ConstLanelet lanelet = test::createLanelet({0.0, 0.0}, {0.0, len}, nb_points);
  SliceRange range;
  range.min_distance = -1.0;
  range.max_distance = -3.0;
  const double slice_length = 1.0;
  const double slice_width = 1.0;
  const int num_right_slice =
    (len / slice_length) * (std::abs(range.max_distance - range.min_distance) / slice_width);
  //
  {
    std::vector<Slice> slices;
    buildSlices(slices, lanelet, range, slice_length, slice_width, 1.0);
    ASSERT_EQ(slices.size(), static_cast<size_t>(num_right_slice));
  }
  // change to the left side (positive distance)
  range.min_distance = 1.0;
  range.max_distance = 3.0;
  {
    std::vector<Slice> slices;
    buildSlices(slices, lanelet, range, slice_length, slice_width, 1.0);
    ASSERT_EQ(slices.size(), static_cast<size_t>(num_right_slice));
  }
}
