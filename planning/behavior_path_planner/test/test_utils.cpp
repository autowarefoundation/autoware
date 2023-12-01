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
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
#include "input.hpp"
#include "lanelet2_core/Attribute.h"
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/Point.h"
#include "lanelet2_core/primitives/Lanelet.h"
#include "motion_utils/trajectory/path_with_lane_id.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

using behavior_path_planner::PathWithLaneId;
using behavior_path_planner::Pose;
using behavior_path_planner::utils::FrenetPoint;
using geometry_msgs::msg::Point;

TEST(BehaviorPathPlanningUtilitiesBehaviorTest, vehiclePoseToFrenetOnStraightLine)
{
  PathWithLaneId path =
    behavior_path_planner::generateStraightSamplePathWithLaneId(0.0f, 1.0f, 10u);
  Pose vehicle_pose = behavior_path_planner::generateEgoSamplePose(10.7f, -1.7f, 0.0);

  const size_t vehicle_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, vehicle_pose, 3.0, 1.0);
  const auto vehicle_pose_frenet = behavior_path_planner::utils::convertToFrenetPoint(
    path.points, vehicle_pose.position, vehicle_seg_idx);

  EXPECT_NEAR(vehicle_pose_frenet.distance, -1.7f, 1e-3);
  EXPECT_NEAR(vehicle_pose_frenet.length, 10.7f, 1e-3);
}

TEST(BehaviorPathPlanningUtilitiesBehaviorTest, vehiclePoseToFrenetOnDiagonalLine)
{
  PathWithLaneId path =
    behavior_path_planner::generateDiagonalSamplePathWithLaneId(0.0f, 1.0f, 10u);
  Pose vehicle_pose = behavior_path_planner::generateEgoSamplePose(0.1f, 0.1f, 0.0);

  const size_t vehicle_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, vehicle_pose, 3.0, 1.0);
  const auto vehicle_pose_frenet = behavior_path_planner::utils::convertToFrenetPoint(
    path.points, vehicle_pose.position, vehicle_seg_idx);

  EXPECT_NEAR(vehicle_pose_frenet.distance, 0, 1e-2);
  EXPECT_NEAR(vehicle_pose_frenet.length, 0.1414f, 1e-2);
}

TEST(BehaviorPathPlanningUtilitiesBehaviorTest, setGoal)
{
  PathWithLaneId path = behavior_path_planner::generateDiagonalSamplePathWithLaneId(0.0f, 1.0f, 5u);
  path.points.at(0).lane_ids.push_back(0);
  path.points.at(1).lane_ids.push_back(1);
  path.points.at(2).lane_ids.push_back(2);
  path.points.at(3).lane_ids.push_back(3);
  path.points.at(3).lane_ids.push_back(4);
  path.points.at(3).lane_ids.push_back(5);
  path.points.at(4).lane_ids.push_back(5);

  PathWithLaneId path_with_goal;
  behavior_path_planner::utils::setGoal(
    3.5, M_PI * 0.5, path, path.points.back().point.pose, 5, &path_with_goal);

  // Check if skipped lane ids by smooth skip connection are filled in output path.
  EXPECT_EQ(path_with_goal.points.size(), 4U);
  ASSERT_THAT(path_with_goal.points.at(0).lane_ids, testing::ElementsAre(0));
  ASSERT_THAT(path_with_goal.points.at(1).lane_ids, testing::ElementsAre(1));
  ASSERT_THAT(path_with_goal.points.at(2).lane_ids, testing::ElementsAre(2, 3, 4, 5));
  ASSERT_THAT(path_with_goal.points.at(3).lane_ids, testing::ElementsAre(5));
}

TEST(BehaviorPathPlanningUtilitiesBehaviorTest, expandLanelets)
{
  using behavior_path_planner::DrivableLanes;
  using behavior_path_planner::utils::expandLanelets;
  std::vector<DrivableLanes> original_lanelets;
  {  // empty list of lanelets, empty output
    const auto expanded_lanelets = expandLanelets(original_lanelets, 0.0, 0.0);
    ASSERT_TRUE(expanded_lanelets.empty());
  }
  double left_bound = 0.0;
  double right_bound = 0.0;
  lanelet::Lanelet lanelet;
  lanelet.leftBound().setAttribute(lanelet::AttributeName::Type, "road_border");
  lanelet.rightBound().setAttribute(lanelet::AttributeName::Type, "road_border");
  lanelet.leftBound().push_back(lanelet::Point3d(lanelet::Id(), 0.0, 1.0));
  lanelet.leftBound().push_back(lanelet::Point3d(lanelet::Id(), 0.0, 2.0));
  lanelet.leftBound().push_back(lanelet::Point3d(lanelet::Id(), 0.0, 3.0));
  lanelet.rightBound().push_back(lanelet::Point3d(lanelet::Id(), 1.0, 1.0));
  lanelet.rightBound().push_back(lanelet::Point3d(lanelet::Id(), 1.0, 2.0));
  lanelet.rightBound().push_back(lanelet::Point3d(lanelet::Id(), 1.0, 3.0));
  DrivableLanes drivable_lane;
  drivable_lane.left_lane = lanelet;
  drivable_lane.right_lane = lanelet;
  original_lanelets.push_back(drivable_lane);
  {  // no offsets, unchanged output
    const auto expanded_lanelets = expandLanelets(original_lanelets, left_bound, right_bound);
    ASSERT_EQ(expanded_lanelets.size(), original_lanelets.size());
    ASSERT_EQ(
      expanded_lanelets[0].left_lane.leftBound().size(),
      original_lanelets[0].left_lane.leftBound().size());
    ASSERT_EQ(
      expanded_lanelets[0].right_lane.rightBound().size(),
      original_lanelets[0].right_lane.rightBound().size());
    for (size_t i = 0; i < expanded_lanelets[0].left_lane.leftBound().size(); ++i) {
      ASSERT_EQ(
        expanded_lanelets[0].left_lane.leftBound()[i].x(),
        original_lanelets[0].left_lane.leftBound()[i].x());
      ASSERT_EQ(
        expanded_lanelets[0].left_lane.leftBound()[i].y(),
        original_lanelets[0].left_lane.leftBound()[i].y());
    }
    for (size_t i = 0; i < expanded_lanelets[0].right_lane.rightBound().size(); ++i) {
      ASSERT_EQ(
        expanded_lanelets[0].right_lane.rightBound()[i].x(),
        original_lanelets[0].right_lane.rightBound()[i].x());
      ASSERT_EQ(
        expanded_lanelets[0].right_lane.rightBound()[i].y(),
        original_lanelets[0].right_lane.rightBound()[i].y());
    }
  }
  left_bound = 0.5;
  right_bound = 1.0;
  {  // skip type, no offset
    const auto expanded_lanelets =
      expandLanelets(original_lanelets, left_bound, right_bound, {"road_border"});
    ASSERT_EQ(expanded_lanelets.size(), original_lanelets.size());
    const auto l_dist = lanelet::geometry::distance2d(
      expanded_lanelets[0].left_lane.leftBound2d(), original_lanelets[0].left_lane.leftBound2d());
    const auto r_dist = lanelet::geometry::distance2d(
      expanded_lanelets[0].right_lane.rightBound2d(),
      original_lanelets[0].right_lane.rightBound2d());
    EXPECT_NEAR(l_dist, 0.0, 1E-03);
    EXPECT_NEAR(r_dist, 0.0, 1E-03);
  }
  {  // expanded lanelet
    const auto expanded_lanelets = expandLanelets(original_lanelets, left_bound, right_bound);
    ASSERT_EQ(expanded_lanelets.size(), original_lanelets.size());
    const auto l_dist = lanelet::geometry::distance2d(
      expanded_lanelets[0].left_lane.leftBound2d(), original_lanelets[0].left_lane.leftBound2d());
    const auto r_dist = lanelet::geometry::distance2d(
      expanded_lanelets[0].right_lane.rightBound2d(),
      original_lanelets[0].right_lane.rightBound2d());
    EXPECT_NEAR(l_dist, left_bound, 1E-03);
    EXPECT_NEAR(r_dist, right_bound, 1E-03);
  }
}
