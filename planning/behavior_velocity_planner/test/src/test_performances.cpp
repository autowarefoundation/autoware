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
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using test::grid_param;

TEST(performances, many_sidewalk_occlusion_spots)
{
  using autoware_auto_planning_msgs::msg::PathWithLaneId;
  using behavior_velocity_planner::occlusion_spot_utils::generatePossibleCollisions;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  std::vector<PossibleCollisionInfo> possible_collisions;

  PathWithLaneId path = test::generatePath(0.5, 0.5, 300.0, 0.5, 300);
  grid_map::GridMap grid = test::generateGrid(300, 300, 0.1);
  for (int x = 0; x < grid.getSize().x(); ++x) {
    for (int y = 0; y < grid.getSize().x(); ++y) {
      grid.at("layer", grid_map::Index(x, y)) =
        behavior_velocity_planner::grid_utils::occlusion_cost_value::UNKNOWN;
    }
  }

  // Set parameters: enable sidewalk obstacles
  behavior_velocity_planner::occlusion_spot_utils::PlannerParam parameters;
  parameters.vehicle_info.baselink_to_front = 0.0;
  parameters.vehicle_info.vehicle_width = 1.0;
  parameters.sidewalk.min_occlusion_spot_size = 1.0;
  parameters.sidewalk.focus_range = 1.0;
  parameters.grid = grid_param;
  const double offset_from_ego_to_closest = 0;
  const double offset_from_closest_to_target = 0;
  std::vector<lanelet::BasicPolygon2d> debug;
  auto start_closest = high_resolution_clock::now();
  generatePossibleCollisions(
    possible_collisions, path, grid, offset_from_ego_to_closest, offset_from_closest_to_target,
    parameters, debug);
  auto end_closest = high_resolution_clock::now();
  std::string ms = " (micro seconds) ";
  std::cout << "| only_closest : runtime (microseconds) \n"
            << "| true : " << duration_cast<microseconds>(end_closest - start_closest).count() << ms
            << "\n";
}
