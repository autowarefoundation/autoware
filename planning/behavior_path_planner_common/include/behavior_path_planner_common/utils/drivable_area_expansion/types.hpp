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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <optional>
#include <vector>

namespace drivable_area_expansion
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

using tier4_autoware_utils::LineString2d;
using tier4_autoware_utils::MultiLineString2d;
using tier4_autoware_utils::MultiPoint2d;
using tier4_autoware_utils::MultiPolygon2d;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using tier4_autoware_utils::Segment2d;

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
}  // namespace drivable_area_expansion
#endif  // BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__TYPES_HPP_
