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

// TEST_F(TestRouteHandler, getClosestLaneletWithinRouteWhenPointsInRoute)
// {
//   lanelet::ConstLanelet closest_lane;

//   Pose search_pose;

//   search_pose.position = tier4_autoware_utils::createPoint(-1.0, 1.75, 0);
//   search_pose.orientation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained7 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained7);
//   ASSERT_EQ(closest_lane.id(), 4775);

//   search_pose.position = tier4_autoware_utils::createPoint(-0.5, 1.75, 0);
//   const auto closest_lane_obtained =
//   search_pose.orientation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained);
//   ASSERT_EQ(closest_lane.id(), 4775);

//   search_pose.position = tier4_autoware_utils::createPoint(-.01, 1.75, 0);
//   search_pose.orientation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained3 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained3);
//   ASSERT_EQ(closest_lane.id(), 4775);

//   search_pose.position = tier4_autoware_utils::createPoint(0.0, 1.75, 0);
//   search_pose.orientation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained1 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained1);
//   ASSERT_EQ(closest_lane.id(), 4775);

//   search_pose.position = tier4_autoware_utils::createPoint(0.01, 1.75, 0);
//   search_pose.orientation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained2 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained2);
//   ASSERT_EQ(closest_lane.id(), 4424);

//   search_pose.position = tier4_autoware_utils::createPoint(0.5, 1.75, 0);
//   search_pose.orientation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained4 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained4);
//   ASSERT_EQ(closest_lane.id(), 4424);

//   search_pose.position = tier4_autoware_utils::createPoint(1.0, 1.75, 0);
//   search_pose.orientation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
//   const auto closest_lane_obtained5 =
//     route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lane);

//   ASSERT_TRUE(closest_lane_obtained5);
//   ASSERT_EQ(closest_lane.id(), 4424);
// }
}  // namespace autoware::route_handler::test
