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
  set_route_handler("overlap_map.osm");
  ASSERT_FALSE(route_handler_->isHandlerReady());

  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose goal_pose;
  start_pose.position = autoware::universe_utils::createPoint(3728.870361, 73739.281250, 0);
  start_pose.orientation = autoware::universe_utils::createQuaternion(0, 0, -0.513117, 0.858319);
  goal_pose.position = autoware::universe_utils::createPoint(3729.961182, 73727.328125, 0);
  goal_pose.orientation = autoware::universe_utils::createQuaternion(0, 0, 0.234831, 0.972036);

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
  set_route_handler("overlap_map.osm");
  set_test_route("overlap_test_route.yaml");
  ASSERT_TRUE(route_handler_->isHandlerReady());

  geometry_msgs::msg::Pose reference_pose;
  geometry_msgs::msg::Pose search_pose;

  lanelet::ConstLanelet reference_lanelet;
  reference_pose.position = autoware::universe_utils::createPoint(3730.88, 73735.3, 0);
  reference_pose.orientation =
    autoware::universe_utils::createQuaternion(0, 0, -0.504626, 0.863338);
  const auto found_reference_lanelet =
    route_handler_->getClosestLaneletWithinRoute(reference_pose, &reference_lanelet);
  ASSERT_TRUE(found_reference_lanelet);
  ASSERT_EQ(reference_lanelet.id(), 168);

  lanelet::ConstLanelet closest_lanelet;
  search_pose.position = autoware::universe_utils::createPoint(3736.89, 73730.8, 0);
  search_pose.orientation = autoware::universe_utils::createQuaternion(0, 0, 0.223244, 0.974763);
  bool found_lanelet = route_handler_->getClosestLaneletWithinRoute(search_pose, &closest_lanelet);
  ASSERT_TRUE(found_lanelet);
  ASSERT_EQ(closest_lanelet.id(), 345);

  found_lanelet = route_handler_->getClosestRouteLaneletFromLanelet(
    search_pose, reference_lanelet, &closest_lanelet, dist_threshold, yaw_threshold);
  ASSERT_TRUE(found_lanelet);
  ASSERT_EQ(closest_lanelet.id(), 277);
}

TEST_F(TestRouteHandler, CheckLaneIsInGoalRouteSection)
{
  const auto lane = route_handler_->getLaneletsFromId(4785);
  const auto is_lane_in_goal_route_section = route_handler_->isInGoalRouteSection(lane);
  ASSERT_TRUE(is_lane_in_goal_route_section);
}

TEST_F(TestRouteHandler, CheckLaneIsNotInGoalRouteSection)
{
  const auto lane = route_handler_->getLaneletsFromId(4780);
  const auto is_lane_in_goal_route_section = route_handler_->isInGoalRouteSection(lane);
  ASSERT_FALSE(is_lane_in_goal_route_section);
}

TEST_F(TestRouteHandler, checkGetLaneletSequence)
{
  const auto current_pose = autoware::test_utils::createPose(-50.0, 1.75, 0.0, 0.0, 0.0, 0.0);

  lanelet::ConstLanelet closest_lanelet;
  const auto found_closest_lanelet = route_handler_->getClosestLaneletWithConstrainsWithinRoute(
    current_pose, &closest_lanelet, dist_threshold, yaw_threshold);
  ASSERT_TRUE(found_closest_lanelet);
  ASSERT_EQ(closest_lanelet.id(), 4765ul);

  const auto current_lanes = route_handler_->getLaneletSequence(
    closest_lanelet, current_pose, backward_path_length, forward_path_length);

  ASSERT_EQ(current_lanes.size(), 6ul);
  ASSERT_EQ(current_lanes.at(0).id(), 4765ul);
  ASSERT_EQ(current_lanes.at(1).id(), 4770ul);
  ASSERT_EQ(current_lanes.at(2).id(), 4775ul);
  ASSERT_EQ(current_lanes.at(3).id(), 4424ul);
  ASSERT_EQ(current_lanes.at(4).id(), 4780ul);
  ASSERT_EQ(current_lanes.at(5).id(), 4785ul);
}

TEST_F(TestRouteHandler, checkLateralIntervalToPreferredLaneWhenLaneChangeToRight)
{
  const auto current_lanes = get_current_lanes();

  // The input is within expectation.
  // this lane is of preferred lane type
  std::for_each(current_lanes.begin(), current_lanes.begin() + 3, [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::RIGHT);
    ASSERT_EQ(result.size(), 0ul);
  });

  // The input is within expectation.
  // this alternative lane is a subset of preferred lane route section
  std::for_each(current_lanes.begin() + 3, current_lanes.end(), [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::RIGHT);
    ASSERT_EQ(result.size(), 1ul);
    EXPECT_DOUBLE_EQ(result.at(0), -3.5);
  });

  // The input is within expectation.
  // Although Direction::NONE, the function should still return result similar to
  // Direction::RIGHT.
  std::for_each(current_lanes.begin(), current_lanes.begin() + 3, [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::NONE);
    ASSERT_EQ(result.size(), 0ul);
  });

  // The input is within expectation.
  // Although Direction::NONE is provided, the function should behave similarly to
  // Direction::RIGHT.
  std::for_each(current_lanes.begin() + 3, current_lanes.end(), [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::NONE);
    ASSERT_EQ(result.size(), 1ul);
    EXPECT_DOUBLE_EQ(result.at(0), -3.5);
  });
}

TEST_F(TestRouteHandler, checkLateralIntervalToPreferredLaneUsingUnexpectedResults)
{
  const auto current_lanes = get_current_lanes();

  std::for_each(current_lanes.begin(), current_lanes.end(), [&](const auto & lane) {
    const auto result = route_handler_->getLateralIntervalsToPreferredLane(lane, Direction::LEFT);
    ASSERT_EQ(result.size(), 0ul);
  });
}

TEST_F(TestRouteHandler, testGetCenterLinePath)
{
  const auto current_lanes = route_handler_->getLaneletsFromIds({4424, 4780, 4785});
  {
    // The input is within expectation.
    const auto center_line_path = route_handler_->getCenterLinePath(current_lanes, 0.0, 50.0);
    ASSERT_EQ(center_line_path.points.size(), 51);  // 26 + 26 - 1(overlapped)
    ASSERT_EQ(center_line_path.points.back().lane_ids.size(), 2);
    ASSERT_EQ(center_line_path.points.back().lane_ids.at(0), 4780);
    ASSERT_EQ(center_line_path.points.back().lane_ids.at(1), 4785);
  }
  {
    // The input is in middle of range.
    const auto center_line_path = route_handler_->getCenterLinePath(current_lanes, 14.5, 60.5);
    ASSERT_EQ(center_line_path.points.size(), 48);
    ASSERT_EQ(center_line_path.points.front().lane_ids.size(), 1);
    ASSERT_EQ(center_line_path.points.back().lane_ids.size(), 1);
    ASSERT_EQ(center_line_path.points.front().lane_ids.at(0), 4424);
    ASSERT_EQ(center_line_path.points.back().lane_ids.at(0), 4785);
  }
  {
    // The input is broken.
    // s_start is negative, and s_end is over the boundary.
    const auto center_line_path = route_handler_->getCenterLinePath(current_lanes, -1.0, 200.0);
    ASSERT_EQ(center_line_path.points.size(), 76);  // 26 + 26 + 26 - 2(overlapped)
    ASSERT_EQ(center_line_path.points.front().lane_ids.size(), 1);
    ASSERT_EQ(center_line_path.points.front().lane_ids.at(0), 4424);
    ASSERT_EQ(center_line_path.points.back().lane_ids.size(), 1);
    ASSERT_EQ(center_line_path.points.back().lane_ids.at(0), 4785);
  }
}
TEST_F(TestRouteHandler, DISABLED_testGetCenterLinePathWhenLanesIsNotConnected)
{
  // broken current lanes. 4424 and 4785 are not connected directly.
  const auto current_lanes = route_handler_->getLaneletsFromIds({4424, 4780, 4785});

  // The input is broken. Test is disabled because it doesn't pass.
  const auto center_line_path = route_handler_->getCenterLinePath(current_lanes, 0.0, 75.0);
  ASSERT_EQ(center_line_path.points.size(), 26);  // 26 + 26 + 26 - 2(overlapped)
  ASSERT_EQ(center_line_path.points.front().lane_ids.size(), 1);
  ASSERT_EQ(center_line_path.points.front().lane_ids.at(0), 4424);
  ASSERT_EQ(center_line_path.points.back().lane_ids.size(), 1);
  ASSERT_EQ(center_line_path.points.back().lane_ids.at(0), 4424);
}

TEST_F(TestRouteHandler, getClosestLaneletWithinRouteWhenPointsInRoute)
{
  auto get_closest_lanelet_within_route =
    [&](double x, double y, double z) -> std::optional<lanelet::Id> {
    const auto pose = autoware::test_utils::createPose(x, y, z, 0.0, 0.0, 0.0);
    lanelet::ConstLanelet closest_lanelet;
    const auto closest_lane_obtained =
      route_handler_->getClosestLaneletWithinRoute(pose, &closest_lanelet);
    if (!closest_lane_obtained) {
      return std::nullopt;
    }
    return closest_lanelet.id();
  };

  ASSERT_TRUE(get_closest_lanelet_within_route(-0.5, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(-0.5, 1.75, 0).value(), 4775ul);

  ASSERT_TRUE(get_closest_lanelet_within_route(-0.01, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(-0.01, 1.75, 0).value(), 4775ul);

  ASSERT_TRUE(get_closest_lanelet_within_route(0.0, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(0.0, 1.75, 0).value(), 4775ul);

  ASSERT_TRUE(get_closest_lanelet_within_route(0.01, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(0.01, 1.75, 0).value(), 4424ul);

  ASSERT_TRUE(get_closest_lanelet_within_route(0.5, 1.75, 0).has_value());
  ASSERT_EQ(get_closest_lanelet_within_route(0.5, 1.75, 0).value(), 4424ul);
}

TEST_F(TestRouteHandler, testGetLaneChangeTargetLanes)
{
  {
    // The input is within expectation.
    // There exist no lane changing lane since both 4770 and 4775 are preferred lane.
    const auto current_lanes = route_handler_->getLaneletsFromIds({4770, 4775});
    const auto lane_change_lane =
      route_handler_->getLaneChangeTarget(current_lanes, Direction::RIGHT);
    ASSERT_FALSE(lane_change_lane.has_value());
  }

  {
    // The input is within expectation.
    // There exist lane changing lane since 4424 is subset of preferred lane 9598.
    const auto current_lanes = route_handler_->getLaneletsFromIds({4775, 4424});
    const auto lane_change_lane =
      route_handler_->getLaneChangeTarget(current_lanes, Direction::RIGHT);
    EXPECT_TRUE(lane_change_lane.has_value());
    ASSERT_EQ(lane_change_lane.value().id(), 9598ul);
  }

  {
    // The input is within expectation.
    // There is a lane-changing lane. Within the maximum current lanes, there is an alternative lane
    // to the preferred lane. Therefore, the lane-changing lane exists.
    const auto current_lanes = get_current_lanes();
    const auto lane_change_lane = route_handler_->getLaneChangeTarget(current_lanes);
    ASSERT_TRUE(lane_change_lane.has_value());
    ASSERT_EQ(lane_change_lane.value().id(), 9598ul);
  }
}

TEST_F(TestRouteHandler, testGetShoulderLaneletsAtPose)
{
  set_route_handler("overlap_map.osm");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 3719.5;
  pose.position.y = 73765.6;
  auto shoulder_lanelets = route_handler_->getShoulderLaneletsAtPose(pose);
  ASSERT_FALSE(shoulder_lanelets.empty());
  ASSERT_EQ(shoulder_lanelets.front().id(), 359ul);

  pose.position.y = 73768.6;
  shoulder_lanelets = route_handler_->getShoulderLaneletsAtPose(pose);
  ASSERT_TRUE(shoulder_lanelets.empty());
}
}  // namespace autoware::route_handler::test
