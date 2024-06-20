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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::drivable_area_expansion
{
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::PathPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

using autoware::universe_utils::LineString2d;
using autoware::universe_utils::MultiLineString2d;
using autoware::universe_utils::MultiPoint2d;
using autoware::universe_utils::MultiPolygon2d;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;
using autoware::universe_utils::Segment2d;

using SegmentRtree = boost::geometry::index::rtree<Segment2d, boost::geometry::index::rstar<16>>;

struct PointDistance
{
  Point2d point{};
  double distance{};
};
struct Projection
{
  Point2d projected_point{};
  double distance{};
  double arc_length{};
};
enum Side { LEFT, RIGHT };
struct Expansion
{
  // mappings from bound index
  std::vector<double> left_distances;
  std::vector<double> right_distances;
  // mappings from path index
  std::vector<size_t> left_bound_indexes;
  std::vector<PointDistance> left_projections;
  std::vector<size_t> right_bound_indexes;
  std::vector<PointDistance> right_projections;
  std::vector<double> min_lane_widths;
};
}  // namespace autoware::behavior_path_planner::drivable_area_expansion
#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_
