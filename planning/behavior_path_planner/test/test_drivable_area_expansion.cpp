// Copyright 2023 TIER IV, Inc.
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

#include "behavior_path_planner/utils/drivable_area_expansion/drivable_area_expansion.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/expansion.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/path_projection.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"
#include "lanelet2_extension/utility/message_conversion.hpp"

#include <gtest/gtest.h>

using drivable_area_expansion::linestring_t;
using drivable_area_expansion::point_t;
using drivable_area_expansion::segment_t;
constexpr auto eps = 1e-9;

TEST(DrivableAreaExpansionProjection, PointToSegment)
{
  using drivable_area_expansion::point_to_segment_projection;

  {
    point_t query(1.0, 1.0);
    segment_t segment(point_t(0.0, 0.0), point_t(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 1.0, eps);
    EXPECT_NEAR(projection.point.x(), 1.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    point_t query(-1.0, 1.0);
    segment_t segment(point_t(0.0, 0.0), point_t(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, std::sqrt(2), eps);
    EXPECT_NEAR(projection.point.x(), 0.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    point_t query(11.0, 1.0);
    segment_t segment(point_t(0.0, 0.0), point_t(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, std::sqrt(2), eps);
    EXPECT_NEAR(projection.point.x(), 10.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    point_t query(5.0, -5.0);
    segment_t segment(point_t(0.0, 0.0), point_t(10.0, 0.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, -5.0, eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 0.0, eps);
  }
  {
    point_t query(5.0, -5.0);
    segment_t segment(point_t(0.0, 0.0), point_t(0.0, -10.0));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 5.0, eps);
    EXPECT_NEAR(projection.point.x(), 0.0, eps);
    EXPECT_NEAR(projection.point.y(), -5.0, eps);
  }
  {
    point_t query(5.0, 5.0);
    segment_t segment(point_t(2.5, 7.5), point_t(7.5, 2.5));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, 0.0, eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 5.0, eps);
  }
  {
    point_t query(0.0, 0.0);
    segment_t segment(point_t(2.5, 7.5), point_t(7.5, 2.5));
    const auto projection = point_to_segment_projection(query, segment.first, segment.second);
    EXPECT_NEAR(projection.distance, -std::sqrt(50), eps);
    EXPECT_NEAR(projection.point.x(), 5.0, eps);
    EXPECT_NEAR(projection.point.y(), 5.0, eps);
  }
}

TEST(DrivableAreaExpansionProjection, PointToLinestring)
{
  using drivable_area_expansion::point_to_linestring_projection;

  linestring_t ls = {
    point_t(0.0, 0.0), point_t(10.0, 0.0), point_t(10.0, 10.0), point_t(0.0, 10.0),
    point_t(5.0, 5.0)};
  {
    point_t query(0.0, 0.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 0.0, eps);
    EXPECT_NEAR(projection.distance, 0.0, eps);
    EXPECT_NEAR(projection.projected_point.x(), 0.0, eps);
    EXPECT_NEAR(projection.projected_point.y(), 0.0, eps);
  }
  {
    point_t query(2.0, 1.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 2.0, eps);
    EXPECT_NEAR(projection.distance, 1.0, eps);
    EXPECT_NEAR(projection.projected_point.x(), 2.0, eps);
    EXPECT_NEAR(projection.projected_point.y(), 0.0, eps);
  }
  {
    point_t query(0.0, 5.0);
    const auto projection = point_to_linestring_projection(query, ls);
    EXPECT_NEAR(projection.arc_length, 30.0 + std::sqrt(2.5 * 2.5 * 2), eps);
    EXPECT_NEAR(projection.distance, -std::sqrt(2.5 * 2.5 * 2), eps);
    EXPECT_NEAR(projection.projected_point.x(), 2.5, eps);
    EXPECT_NEAR(projection.projected_point.y(), 7.5, eps);
  }
}

TEST(DrivableAreaExpansionProjection, LinestringToPoint)
{
  using drivable_area_expansion::linestring_to_point_projection;

  linestring_t ls = {
    point_t(0.0, 0.0), point_t(10.0, 0.0), point_t(10.0, 10.0), point_t(0.0, 10.0),
    point_t(5.0, 5.0)};
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

TEST(DrivableAreaExpansionProjection, SubLinestring)
{
  using drivable_area_expansion::sub_linestring;

  const linestring_t ls = {
    point_t{0.0, 0.0}, point_t{1.0, 0.0}, point_t{2.0, 0.0}, point_t{3.0, 0.0},
    point_t{4.0, 0.0}, point_t{5.0, 0.0}, point_t{6.0, 0.0},
  };
  {
    // arc lengths equal to the original range: same linestring is returned
    const auto sub = sub_linestring(ls, 0.0, 6.0);
    ASSERT_EQ(ls.size(), sub.size());
    for (auto i = 0lu; i < ls.size(); ++i) EXPECT_TRUE(boost::geometry::equals(ls[i], sub[i]));
  }
  {
    // arc lengths equal to existing point: sub-linestring with same points
    const auto sub = sub_linestring(ls, 1.0, 5.0);
    ASSERT_EQ(ls.size() - 2lu, sub.size());
    for (auto i = 0lu; i < sub.size(); ++i) EXPECT_TRUE(boost::geometry::equals(ls[i + 1], sub[i]));
  }
  {
    // arc lengths inside the original: sub-linestring with some interpolated points
    const auto sub = sub_linestring(ls, 1.5, 2.5);
    ASSERT_EQ(sub.size(), 3lu);
    EXPECT_NEAR(sub[0].x(), 1.5, eps);
    EXPECT_NEAR(sub[1].x(), 2.0, eps);
    EXPECT_NEAR(sub[2].x(), 2.5, eps);
    for (const auto & p : sub) EXPECT_NEAR(p.y(), 0.0, eps);
  }
  {
    // arc length outside of the original range: first & last point are replaced by interpolations
    const auto sub = sub_linestring(ls, -0.5, 8.5);
    ASSERT_EQ(sub.size(), ls.size());
    EXPECT_NEAR(sub.front().x(), -0.5, eps);
    for (auto i = 1lu; i + 1 < ls.size(); ++i) EXPECT_TRUE(boost::geometry::equals(ls[i], sub[i]));
    EXPECT_NEAR(sub.back().x(), 8.5, eps);
    for (const auto & p : sub) EXPECT_NEAR(p.y(), 0.0, eps);
  }
}

TEST(DrivableAreaExpansionProjection, InverseProjection)
{
  using drivable_area_expansion::linestring_to_point_projection;
  using drivable_area_expansion::point_to_linestring_projection;

  linestring_t ls = {
    point_t(0.0, 0.0), point_t(10.0, 0.0), point_t(10.0, 10.0), point_t(0.0, 10.0),
    point_t(5.0, 5.0)};

  for (auto x = 0.0; x < 10.0; x += 0.1) {
    for (auto y = 0.0; x < 10.0; x += 0.1) {
      point_t p(x, y);
      const auto projection = point_to_linestring_projection(p, ls);
      const auto inverse =
        linestring_to_point_projection(ls, projection.arc_length, projection.distance);
      EXPECT_NEAR(inverse.second.x(), p.x(), eps);
      EXPECT_NEAR(inverse.second.y(), p.y(), eps);
    }
  }
}

TEST(DrivableAreaExpansionProjection, expandDrivableArea)
{
  drivable_area_expansion::DrivableAreaExpansionParameters params;
  drivable_area_expansion::PredictedObjects dynamic_objects;
  autoware_auto_mapping_msgs::msg::HADMapBin map;
  lanelet::LaneletMapPtr empty_lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::toBinMsg(empty_lanelet_map_ptr, &map);
  route_handler::RouteHandler route_handler(map);
  lanelet::ConstLanelets path_lanes = {};
  drivable_area_expansion::PathWithLaneId path;
  {  // Simple path with Y=0 and X = 0, 1, 2
    drivable_area_expansion::PathWithLaneId::_points_type::value_type p;
    p.point.pose.position.x = 0.0;
    p.point.pose.position.y = 0.0;
    path.points.push_back(p);
    p.point.pose.position.x = 1.0;
    path.points.push_back(p);
    p.point.pose.position.x = 2.0;
    path.points.push_back(p);
  }
  {  // Left bound at Y = 1, Right bound at Y = -1
    drivable_area_expansion::Point pl;
    drivable_area_expansion::Point pr;
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
    params.compensate_extra_dist = false;
    params.max_expansion_distance = 0.0;  // means no limit
    params.max_path_arc_length = 0.0;     // means no limit
    params.extra_arc_length = 1.0;
    params.expansion_method = "polygon";
    // 2m x 4m ego footprint
    params.ego_front_offset = 1.0;
    params.ego_rear_offset = -1.0;
    params.ego_left_offset = 2.0;
    params.ego_right_offset = -2.0;
  }
  // we expect the drivable area to be expanded by 1m on each side
  // BUT short paths, due to pruning at the edge of the driving area, there is no expansion
  drivable_area_expansion::expandDrivableArea(
    path, params, dynamic_objects, route_handler, path_lanes);
  // unchanged path points
  ASSERT_EQ(path.points.size(), 3ul);
  for (auto i = 0.0; i < path.points.size(); ++i) {
    EXPECT_NEAR(path.points[i].point.pose.position.x, i, eps);
    EXPECT_NEAR(path.points[i].point.pose.position.y, 0.0, eps);
  }

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
}

TEST(DrivableAreaExpansion, calculateDistanceLimit)
{
  using drivable_area_expansion::calculateDistanceLimit;
  using drivable_area_expansion::linestring_t;
  using drivable_area_expansion::multi_linestring_t;
  using drivable_area_expansion::polygon_t;

  {
    const linestring_t base_ls = {{0.0, 0.0}, {10.0, 0.0}};
    const multi_linestring_t uncrossable_lines = {};
    const polygon_t expansion_polygon = {
      {{0.0, -4.0}, {0.0, 4.0}, {10.0, 4.0}, {10.0, -4.0}, {10.0, -4.0}}, {}};
    const auto limit_distance =
      calculateDistanceLimit(base_ls, expansion_polygon, uncrossable_lines);
    EXPECT_NEAR(limit_distance, std::numeric_limits<double>::max(), 1e-9);
  }
  {
    const linestring_t base_ls = {{0.0, 0.0}, {10.0, 0.0}};
    const linestring_t uncrossable_line = {{0.0, 2.0}, {10.0, 2.0}};
    const polygon_t expansion_polygon = {
      {{0.0, -4.0}, {0.0, 4.0}, {10.0, 4.0}, {10.0, -4.0}, {10.0, -4.0}}, {}};
    const auto limit_distance =
      calculateDistanceLimit(base_ls, expansion_polygon, {uncrossable_line});
    EXPECT_NEAR(limit_distance, 2.0, 1e-9);
  }
  {
    const linestring_t base_ls = {{0.0, 0.0}, {10.0, 0.0}};
    const multi_linestring_t uncrossable_lines = {
      {{0.0, 2.0}, {10.0, 2.0}}, {{0.0, 1.5}, {10.0, 1.0}}};
    const polygon_t expansion_polygon = {
      {{0.0, -4.0}, {0.0, 4.0}, {10.0, 4.0}, {10.0, -4.0}, {10.0, -4.0}}, {}};
    const auto limit_distance =
      calculateDistanceLimit(base_ls, expansion_polygon, uncrossable_lines);
    EXPECT_NEAR(limit_distance, 1.0, 1e-9);
  }
}
