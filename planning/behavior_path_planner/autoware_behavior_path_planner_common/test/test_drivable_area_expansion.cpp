// Copyright 2023-2024 TIER IV, Inc.
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

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/drivable_area_expansion.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/path_projection.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/types.hpp"
#include "autoware_lanelet2_extension/utility/message_conversion.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

using autoware::behavior_path_planner::drivable_area_expansion::LineString2d;
using autoware::behavior_path_planner::drivable_area_expansion::Point2d;
using autoware::behavior_path_planner::drivable_area_expansion::Segment2d;
constexpr auto eps = 1e-9;

TEST(DrivableAreaExpansionProjection, PointToSegment)
{
  using autoware::behavior_path_planner::drivable_area_expansion::point_to_segment_projection;

  {
    Point2d query(1.0, 1.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 1.0, eps);
    EXPECT_NEAR(projection.point.x(), 1.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    Point2d query(-1.0, 1.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, std::sqrt(2), eps);
    EXPECT_NEAR(projection.point.x(), 0.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    Point2d query(11.0, 1.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, std::sqrt(2), eps);
    EXPECT_NEAR(projection.point.x(), 10.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    Point2d query(5.0, -5.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 5.0, eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    Point2d query(5.0, -5.0);
    Segment2d segment(Point2d(0.0, 0.0), Point2d(0.0, -10.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 5.0, eps);
    EXPECT_NEAR(projection.point.x(), 0.0, eps);
    EXPECT_NEAR(projection.point.y(), -5.0, eps);
  }
  {
    Point2d query(5.0, 5.0);
    Segment2d segment(Point2d(2.5, 7.5), Point2d(7.5, 2.5));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 0.0, eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 5.0, eps);
  }
  {
    Point2d query(0.0, 0.0);
    Segment2d segment(Point2d(2.5, 7.5), Point2d(7.5, 2.5));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, std::sqrt(50), eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 5.0, eps);
  }
}

TEST(DrivableAreaExpansionProjection, PointToLinestring)
{
  using autoware::behavior_path_planner::drivable_area_expansion::point_to_linestring_projection;

  LineString2d ls = {
    Point2d(0.0, 0.0), Point2d(10.0, 0.0), Point2d(10.0, 10.0), Point2d(0.0, 10.0),
    Point2d(5.0, 5.0)};
  {
    Point2d query(0.0, 0.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 0.0, eps);
    EXPECT_NEAR(projection.distance, 0.0, eps);
    EXPECT_NEAR(projection.projected_point.x(), 0.0, eps);
    EXPECT_NEAR(projection.projected_point.y(), 0.0, eps);
  }
  {
    Point2d query(2.0, 1.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 2.0, eps);
    EXPECT_NEAR(projection.distance, 1.0, eps);
    EXPECT_NEAR(projection.projected_point.x(), 2.0, eps);
    EXPECT_NEAR(projection.projected_point.y(), 0.0, eps);
  }
  {
    Point2d query(0.0, 5.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 30.0 + std::sqrt(2.5 * 2.5 * 2), eps);
    EXPECT_NEAR(projection.distance, std::sqrt(2.5 * 2.5 * 2), eps);
    EXPECT_NEAR(projection.projected_point.x(), 2.5, eps);
    EXPECT_NEAR(projection.projected_point.y(), 7.5, eps);
  }
}

TEST(DrivableAreaExpansionProjection, LinestringToPoint)
{
  using autoware::behavior_path_planner::drivable_area_expansion::linestring_to_point_projection;

  LineString2d ls = {
    Point2d(0.0, 0.0), Point2d(10.0, 0.0), Point2d(10.0, 10.0), Point2d(0.0, 10.0),
    Point2d(5.0, 5.0)};
  for (auto arc_length = 0.0; arc_length <= 10.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_NEAR(projection.first.x(), arc_length, eps);
    EXPECT_NEAR(projection.first.y(), 0.0, eps);
    EXPECT_NEAR(projection.second.x(), arc_length, eps);
    EXPECT_NEAR(projection.second.y(), 0.0, eps);
  }
  for (auto arc_length = 11.0; arc_length <= 20.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_NEAR(projection.first.x(), 10.0, eps);
    EXPECT_NEAR(projection.first.y(), arc_length - 10.0, eps);
    EXPECT_NEAR(projection.second.x(), 10.0, eps);
    EXPECT_NEAR(projection.second.y(), arc_length - 10.0, eps);
  }
  for (auto arc_length = 21.0; arc_length <= 30.0; arc_length += 1.0) {
    const auto projection = linestring_to_point_projection(ls, arc_length, 0.0);
    EXPECT_NEAR(projection.first.x(), 10.0 + (20 - arc_length), eps);
    EXPECT_NEAR(projection.first.y(), 10.0, eps);
    EXPECT_NEAR(projection.second.x(), 10.0 + (20 - arc_length), eps);
    EXPECT_NEAR(projection.second.y(), 10.0, eps);
  }
}

TEST(DrivableAreaExpansionProjection, InverseProjection)
{
  using autoware::behavior_path_planner::drivable_area_expansion::linestring_to_point_projection;
  using autoware::behavior_path_planner::drivable_area_expansion::point_to_linestring_projection;

  LineString2d ls = {
    Point2d(0.0, 0.0), Point2d(10.0, 0.0), Point2d(10.0, 10.0), Point2d(0.0, 10.0),
    Point2d(5.0, 5.0)};

  for (auto x = 0.0; x < 10.0; x += 0.1) {
    for (auto y = 0.0; x < 10.0; x += 0.1) {
      Point2d p(x, y);
      const auto projection = point_to_linestring_projection(p, ls);
      const auto inverse =
        linestring_to_point_projection(ls, projection.arc_length, projection.distance);
      EXPECT_NEAR(inverse.second.x(), p.x(), eps);
      EXPECT_NEAR(inverse.second.y(), p.y(), eps);
    }
  }
}

TEST(DrivableAreaExpansionProjection, expand_drivable_area)
{
  autoware::behavior_path_planner::drivable_area_expansion::DrivableAreaExpansionParameters params;
  autoware::behavior_path_planner::drivable_area_expansion::PredictedObjects dynamic_objects;
  autoware_map_msgs::msg::LaneletMapBin map;
  lanelet::LaneletMapPtr empty_lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::toBinMsg(empty_lanelet_map_ptr, &map);
  autoware::route_handler::RouteHandler route_handler(map);
  lanelet::ConstLanelets path_lanes = {};
  autoware::behavior_path_planner::drivable_area_expansion::PathWithLaneId path;
  {  // Simple path with Y=0 and X = 0, 1, 2
    autoware::behavior_path_planner::drivable_area_expansion::PathWithLaneId::_points_type::
      value_type p;
    p.point.pose.position.x = 0.0;
    p.point.pose.position.y = 0.0;
    path.points.push_back(p);
    p.point.pose.position.x = 1.0;
    path.points.push_back(p);
    p.point.pose.position.x = 2.0;
    path.points.push_back(p);
  }
  {  // Left bound at Y = 1, Right bound at Y = -1
    autoware::behavior_path_planner::drivable_area_expansion::Point pl;
    autoware::behavior_path_planner::drivable_area_expansion::Point pr;
    pl.y = 1.0;
    pr.y = -1.0;
    pl.x = 0.0;
    pr.x = 0.0;
    path.left_bound.push_back(pl);
    path.right_bound.push_back(pr);
    pl.x = 1.0;
    pr.x = 1.0;
    path.left_bound.push_back(pl);
    path.right_bound.push_back(pr);
    pl.x = 2.0;
    pr.x = 2.0;
    path.left_bound.push_back(pl);
    path.right_bound.push_back(pr);
  }
  {  // parameters
    params.enabled = true;
    params.avoid_dynamic_objects = false;
    params.avoid_linestring_dist = 0.0;
    params.avoid_linestring_types = {};
    params.max_expansion_distance = 0.0;  // means no limit
    params.max_path_arc_length = 0.0;     // means no limit
    params.resample_interval = 1.0;
    // 2m x 4m ego footprint
    params.vehicle_info.front_overhang_m = 0.0;
    params.vehicle_info.wheel_base_m = 2.0;
    params.vehicle_info.vehicle_width_m = 2.0;
  }
  autoware::behavior_path_planner::PlannerData planner_data;
  planner_data.drivable_area_expansion_parameters = params;
  planner_data.dynamic_object =
    std::make_shared<autoware::behavior_path_planner::drivable_area_expansion::PredictedObjects>(
      dynamic_objects);
  planner_data.self_odometry = std::make_shared<nav_msgs::msg::Odometry>();
  planner_data.route_handler =
    std::make_shared<autoware::route_handler::RouteHandler>(route_handler);
  autoware::behavior_path_planner::drivable_area_expansion::expand_drivable_area(
    path, std::make_shared<autoware::behavior_path_planner::PlannerData>(planner_data));
  // unchanged path points
  ASSERT_EQ(path.points.size(), 3ul);
  for (auto i = 0.0; i < path.points.size(); ++i) {
    EXPECT_NEAR(path.points[i].point.pose.position.x, i, eps);
    EXPECT_NEAR(path.points[i].point.pose.position.y, 0.0, eps);
  }
  // straight path: no expansion
  // expanded left bound
  ASSERT_EQ(path.left_bound.size(), 3ul);
  EXPECT_NEAR(path.left_bound[0].x, 0.0, eps);
  EXPECT_NEAR(path.left_bound[0].y, 1.0, eps);
  EXPECT_NEAR(path.left_bound[1].x, 1.0, eps);
  EXPECT_NEAR(path.left_bound[1].y, 1.0, eps);
  EXPECT_NEAR(path.left_bound[2].x, 2.0, eps);
  EXPECT_NEAR(path.left_bound[2].y, 1.0, eps);
  // expanded right bound
  ASSERT_EQ(path.right_bound.size(), 3ul);
  EXPECT_NEAR(path.right_bound[0].x, 0.0, eps);
  EXPECT_NEAR(path.right_bound[0].y, -1.0, eps);
  EXPECT_NEAR(path.right_bound[1].x, 1.0, eps);
  EXPECT_NEAR(path.right_bound[1].y, -1.0, eps);
  EXPECT_NEAR(path.right_bound[2].x, 2.0, eps);
  EXPECT_NEAR(path.right_bound[2].y, -1.0, eps);

  // add some curvature
  path.points[1].point.pose.position.y = 0.5;

  autoware::behavior_path_planner::drivable_area_expansion::expand_drivable_area(
    path, std::make_shared<autoware::behavior_path_planner::PlannerData>(planner_data));
  // expanded left bound
  for (const auto & p : path.left_bound) {
    EXPECT_GT(p.y, 1.0);
  }
  // expanded right bound
  for (const auto & p : path.right_bound) {
    EXPECT_LT(p.y, -1.0);
  }
}
