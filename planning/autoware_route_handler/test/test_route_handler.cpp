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

#include "test_route_handler.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace autoware::route_handler::test
{
TEST_F(TestRouteHandler, isRouteHandlerReadyTest)
{
  ASSERT_TRUE(route_handler_->isHandlerReady());
}

TEST_F(TestRouteHandler, checkIfIDReturned)
{
  const auto lanelet1 = route_handler_->getLaneletsFromId(4785);
  const auto is_lanelet1_in_goal_route_section = route_handler_->isInGoalRouteSection(lanelet1);
  ASSERT_TRUE(is_lanelet1_in_goal_route_section);

  const auto lanelet2 = route_handler_->getLaneletsFromId(4780);
  const auto is_lanelet2_in_goal_route_section = route_handler_->isInGoalRouteSection(lanelet2);
  ASSERT_FALSE(is_lanelet2_in_goal_route_section);
}

TEST_F(TestRouteHandler, getGoalLaneId)
{
  lanelet::ConstLanelet goal_lane;

  const auto goal_lane_obtained = route_handler_->getGoalLanelet(&goal_lane);
  ASSERT_TRUE(goal_lane_obtained);
  ASSERT_EQ(goal_lane.id(), 5088);
}

TEST_F(TestRouteHandler, getLaneletSequenceWhenOverlappingRoute)
{
  set_route_handler("/test_map/overlap_map.osm");
  ASSERT_FALSE(route_handler_->isHandlerReady());

  geometry_msgs::msg::Pose start_pose, goal_pose;
  start_pose.position = autoware_universe_utils::createPoint(3728.870361, 73739.281250, 0);
  start_pose.orientation = autoware_universe_utils::createQuaternion(0, 0, -0.513117, 0.858319);
  goal_pose.position = autoware_universe_utils::createPoint(3729.961182, 73727.328125, 0);
  goal_pose.orientation = autoware_universe_utils::createQuaternion(0, 0, 0.234831, 0.972036);

  lanelet::ConstLanelets path_lanelets;
  ASSERT_TRUE(
    route_handler_->planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets));
  ASSERT_EQ(path_lanelets.size(), 12);
  ASSERT_EQ(path_lanelets.front().id(), 168);
  ASSERT_EQ(path_lanelets.back().id(), 345);

  route_handler_->setRouteLanelets(path_lanelets);
  ASSERT_TRUE(route_handler_->isHandlerReady());

  rclcpp::init(0, nullptr);
  ASSERT_TRUE(rclcpp::ok());

  auto lanelet_sequence = route_handler_->getLaneletSequence(path_lanelets.back());
  ASSERT_EQ(lanelet_sequence.size(), 12);
  ASSERT_EQ(lanelet_sequence.front().id(), 168);
  ASSERT_EQ(lanelet_sequence.back().id(), 345);
}

TEST_F(TestRouteHandler, getClosestRouteLaneletFromLaneletWhenOverlappingRoute)
{
  set_route_handler("/test_map/overlap_map.osm");
  set_test_route("/test_route/overlap_test_route.yaml");
  ASSERT_TRUE(route_handler_->isHandlerReady());

  geometry_msgs::msg::Pose reference_pose, search_pose;

  lanelet::ConstLanelet reference_lanelet;
  reference_pose.position = autoware_universe_utils::createPoint(3730.88, 73735.3, 0);
  reference_pose.orientation = autoware_universe_utils::createQuaternion(0, 0, -0.504626, 0.863338);
  const auto found_reference_lanelet =
    route_handler_->getClosestLaneletWithinRoute(reference_pose, &reference_lanelet);
  ASSERT_TRUE(found_reference_lanelet);
  ASSERT_EQ(reference_lanelet.id(), 168);

  lanelet::ConstLanelet closest_lanelet;
  search_pose.position = autoware_universe_utils::createPoint(3736.89, 73730.8, 0);
  search_pose.orientation = autoware_universe_utils::createQuaternion(0, 0, 0.223244, 0.974763);
  bool found_lanelet = route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lanelet);
  ASSERT_TRUE(found_lanelet);
  ASSERT_EQ(closest_lanelet.id(), 345);

  found_lanelet = route_handler_->getClosestRouteLaneletFromLanelet(
    search_pose, reference_lanelet, &closest_lanelet, 3.0, 1.046);
  ASSERT_TRUE(found_lanelet);
  ASSERT_EQ(closest_lanelet.id(), 277);
}

// TEST_F(TestRouteHandler, getClosestLaneletWithinRouteWhenPointsInRoute)
// {
//   lanelet::ConstLanelet closest_lane;

//   Pose search_pose;

//   search_pose.position = autoware_universe_utils::createPoint(-1.0, 1.75, 0);
//   search_pose.orientation = autoware_universe_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained7 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained7);
//   ASSERT_EQ(closest_lane.id(), 4775);

//   search_pose.position = autoware_universe_utils::createPoint(-0.5, 1.75, 0);
//   const auto closest_lane_obtained =
//   search_pose.orientation = autoware_universe_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained);
//   ASSERT_EQ(closest_lane.id(), 4775);

//   search_pose.position = autoware_universe_utils::createPoint(-.01, 1.75, 0);
//   search_pose.orientation = autoware_universe_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained3 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained3);
//   ASSERT_EQ(closest_lane.id(), 4775);

//   search_pose.position = autoware_universe_utils::createPoint(0.0, 1.75, 0);
//   search_pose.orientation = autoware_universe_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained1 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained1);
//   ASSERT_EQ(closest_lane.id(), 4775);

//   search_pose.position = autoware_universe_utils::createPoint(0.01, 1.75, 0);
//   search_pose.orientation = autoware_universe_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained2 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained2);
//   ASSERT_EQ(closest_lane.id(), 4424);

//   search_pose.position = autoware_universe_utils::createPoint(0.5, 1.75, 0);
//   search_pose.orientation = autoware_universe_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained4 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained4);
//   ASSERT_EQ(closest_lane.id(), 4424);

//   search_pose.position = autoware_universe_utils::createPoint(1.0, 1.75, 0);
//   search_pose.orientation = autoware_universe_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained5 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained5);
//   ASSERT_EQ(closest_lane.id(), 4424);
// }
}  // namespace autoware::route_handler::test
