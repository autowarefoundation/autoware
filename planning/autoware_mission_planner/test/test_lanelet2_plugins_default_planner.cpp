// Copyright 2024 TIER IV
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

#include <../src/lanelet2_plugins/default_planner.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <boost/geometry/io/wkt/write.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

struct DefaultPlanner : protected autoware::mission_planner::lanelet2::DefaultPlanner
{
  void set_front_offset(const double offset) { vehicle_info_.max_longitudinal_offset_m = offset; }
  void set_dummy_map() { route_handler_.setMap(autoware::test_utils::makeMapBinMsg()); }
  [[nodiscard]] bool check_goal_inside_lanes(
    const lanelet::ConstLanelet & closest_lanelet_to_goal,
    const lanelet::ConstLanelets & path_lanelets,
    const autoware::universe_utils::Polygon2d & goal_footprint) const
  {
    return check_goal_footprint_inside_lanes(
      closest_lanelet_to_goal, path_lanelets, goal_footprint);
  }
};

TEST(TestLanelet2PluginsDefaultPlanner, checkGoalInsideLane)
{
  // Test with dummy map such that only the lanelets provided as inputs are used for the ego lane
  DefaultPlanner planner;
  planner.set_dummy_map();
  // vehicle max longitudinal offset is used to retrieve additional lanelets from the map
  planner.set_front_offset(0.0);
  lanelet::LineString3d left_bound;
  lanelet::LineString3d right_bound;
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, -1});
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, -1});
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, -1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, 1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, 1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, 1});
  lanelet::ConstLanelet goal_lanelet{lanelet::InvalId, left_bound, right_bound};

  // simple case where the footprint is completely inside the lane
  autoware::universe_utils::Polygon2d goal_footprint;
  goal_footprint.outer().emplace_back(0, 0);
  goal_footprint.outer().emplace_back(0, 0.5);
  goal_footprint.outer().emplace_back(0.5, 0.5);
  goal_footprint.outer().emplace_back(0.5, 0);
  goal_footprint.outer().emplace_back(0, 0);
  EXPECT_TRUE(planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet}, goal_footprint));

  // the footprint touches the border of the lanelet
  goal_footprint.clear();
  goal_footprint.outer().emplace_back(0, 0);
  goal_footprint.outer().emplace_back(0, 1);
  goal_footprint.outer().emplace_back(1, 1);
  goal_footprint.outer().emplace_back(1, 0);
  goal_footprint.outer().emplace_back(0, 0);
  EXPECT_TRUE(planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet}, goal_footprint));

  // add lanelets such that the footprint touches the linestring shared by the combined lanelets
  lanelet::LineString3d next_left_bound;
  lanelet::LineString3d next_right_bound;
  next_left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, -1});
  next_left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 2, -1});
  next_right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, 1});
  next_right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 2, 1});
  lanelet::ConstLanelet next_lanelet{lanelet::InvalId, next_left_bound, next_right_bound};
  EXPECT_TRUE(
    planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet, next_lanelet}, goal_footprint));

  // the footprint is inside the other lanelet
  goal_footprint.clear();
  goal_footprint.outer().emplace_back(1.1, -0.5);
  goal_footprint.outer().emplace_back(1.1, 0.5);
  goal_footprint.outer().emplace_back(1.6, 0.5);
  goal_footprint.outer().emplace_back(1.6, -0.5);
  goal_footprint.outer().emplace_back(1.1, -0.5);
  EXPECT_TRUE(
    planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet, next_lanelet}, goal_footprint));

  // the footprint is completely outside of the lanelets
  goal_footprint.clear();
  goal_footprint.outer().emplace_back(1.1, 1.5);
  goal_footprint.outer().emplace_back(1.1, 2.0);
  goal_footprint.outer().emplace_back(1.6, 2.0);
  goal_footprint.outer().emplace_back(1.6, 1.5);
  goal_footprint.outer().emplace_back(1.1, 1.5);
  EXPECT_FALSE(
    planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet, next_lanelet}, goal_footprint));

  // the footprint is outside of the lanelets but touches an edge
  goal_footprint.clear();
  goal_footprint.outer().emplace_back(1.1, 1.0);
  goal_footprint.outer().emplace_back(1.1, 2.0);
  goal_footprint.outer().emplace_back(1.6, 2.0);
  goal_footprint.outer().emplace_back(1.6, 1.0);
  goal_footprint.outer().emplace_back(1.1, 1.0);
  EXPECT_FALSE(
    planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet, next_lanelet}, goal_footprint));

  // the footprint is outside of the lanelets but share a point
  goal_footprint.clear();
  goal_footprint.outer().emplace_back(2.0, 1.0);
  goal_footprint.outer().emplace_back(2.0, 2.0);
  goal_footprint.outer().emplace_back(3.0, 2.0);
  goal_footprint.outer().emplace_back(3.0, 1.0);
  goal_footprint.outer().emplace_back(2.0, 1.0);
  EXPECT_FALSE(
    planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet, next_lanelet}, goal_footprint));

  // ego footprint that overlaps both lanelets
  goal_footprint.clear();
  goal_footprint.outer().emplace_back(-0.5, -0.5);
  goal_footprint.outer().emplace_back(-0.5, 0.5);
  goal_footprint.outer().emplace_back(1.5, 0.5);
  goal_footprint.outer().emplace_back(1.5, -0.5);
  goal_footprint.outer().emplace_back(-0.5, -0.5);
  EXPECT_TRUE(
    planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet, next_lanelet}, goal_footprint));

  // ego footprint that goes further than the next lanelet
  goal_footprint.clear();
  goal_footprint.outer().emplace_back(-0.5, -0.5);
  goal_footprint.outer().emplace_back(-0.5, 0.5);
  goal_footprint.outer().emplace_back(2.5, 0.5);
  goal_footprint.outer().emplace_back(2.5, -0.5);
  goal_footprint.outer().emplace_back(-0.5, -0.5);
  EXPECT_FALSE(
    planner.check_goal_inside_lanes(goal_lanelet, {goal_lanelet, next_lanelet}, goal_footprint));
}
