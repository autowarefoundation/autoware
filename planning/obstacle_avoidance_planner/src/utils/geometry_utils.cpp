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

#include "obstacle_avoidance_planner/utils/geometry_utils.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "obstacle_avoidance_planner/mpt_optimizer.hpp"
#include "tf2/utils.h"

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <stack>
#include <vector>

namespace obstacle_avoidance_planner
{
namespace bg = boost::geometry;
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::LineString2d;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

namespace
{
geometry_msgs::msg::Point getStartPoint(
  const std::vector<geometry_msgs::msg::Point> & bound, const geometry_msgs::msg::Point & point)
{
  const size_t segment_idx = motion_utils::findNearestSegmentIndex(bound, point);
  const auto & curr_seg_point = bound.at(segment_idx);
  const auto & next_seg_point = bound.at(segment_idx);
  const Eigen::Vector2d first_to_target{point.x - curr_seg_point.x, point.y - curr_seg_point.y};
  const Eigen::Vector2d first_to_second{
    next_seg_point.x - curr_seg_point.x, next_seg_point.y - curr_seg_point.y};
  const double length = first_to_target.dot(first_to_second.normalized());

  if (length < 0.0) {
    return bound.front();
  }

  const auto first_point = motion_utils::calcLongitudinalOffsetPoint(bound, segment_idx, length);
  if (first_point) {
    return *first_point;
  }

  return bound.front();
}

bool isFrontDrivableArea(
  const geometry_msgs::msg::Point & point,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  if (left_bound.empty() || right_bound.empty()) {
    return false;
  }

  constexpr double min_dist = 0.1;
  const auto left_start_point = getStartPoint(left_bound, right_bound.front());
  const auto right_start_point = getStartPoint(right_bound, left_bound.front());

  // ignore point behind of the front line
  const std::vector<geometry_msgs::msg::Point> front_bound = {left_start_point, right_start_point};
  const double lat_dist_to_front_bound = motion_utils::calcLateralOffset(front_bound, point);
  if (lat_dist_to_front_bound < min_dist) {
    return true;
  }

  return false;
}

Polygon2d createDrivablePolygon(
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound)
{
  Polygon2d drivable_area_poly;

  // left bound
  for (const auto & p : left_bound) {
    drivable_area_poly.outer().push_back(Point2d(p.x, p.y));
  }

  // right bound
  auto reversed_right_bound = right_bound;
  std::reverse(reversed_right_bound.begin(), reversed_right_bound.end());
  for (const auto & p : reversed_right_bound) {
    drivable_area_poly.outer().push_back(Point2d(p.x, p.y));
  }

  drivable_area_poly.outer().push_back(drivable_area_poly.outer().front());
  return drivable_area_poly;
}
}  // namespace

namespace geometry_utils
{
bool isOutsideDrivableAreaFromRectangleFootprint(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const vehicle_info_util::VehicleInfo & vehicle_info,
  const bool use_footprint_polygon_for_outside_drivable_area_check)
{
  if (left_bound.empty() || right_bound.empty()) {
    return false;
  }

  const double base_to_right = vehicle_info.wheel_tread_m / 2.0 + vehicle_info.right_overhang_m;
  const double base_to_left = vehicle_info.wheel_tread_m / 2.0 + vehicle_info.left_overhang_m;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  // calculate footprint corner points
  const auto top_left_pos =
    tier4_autoware_utils::calcOffsetPose(pose, base_to_front, base_to_left, 0.0).position;
  const auto top_right_pos =
    tier4_autoware_utils::calcOffsetPose(pose, base_to_front, -base_to_right, 0.0).position;
  const auto bottom_right_pos =
    tier4_autoware_utils::calcOffsetPose(pose, -base_to_rear, -base_to_right, 0.0).position;
  const auto bottom_left_pos =
    tier4_autoware_utils::calcOffsetPose(pose, -base_to_rear, base_to_left, 0.0).position;

  if (use_footprint_polygon_for_outside_drivable_area_check) {
    // calculate footprint polygon
    LinearRing2d footprint_polygon;
    footprint_polygon.push_back({top_left_pos.x, top_left_pos.y});
    footprint_polygon.push_back({top_right_pos.x, top_right_pos.y});
    footprint_polygon.push_back({bottom_right_pos.x, bottom_right_pos.y});
    footprint_polygon.push_back({bottom_left_pos.x, bottom_left_pos.y});
    bg::correct(footprint_polygon);

    // calculate boundary line strings
    LineString2d left_bound_line;
    for (const auto & p : left_bound) {
      left_bound_line.push_back({p.x, p.y});
    }

    LineString2d right_bound_line;
    for (const auto & p : right_bound) {
      right_bound_line.push_back({p.x, p.y});
    }

    LineString2d back_bound_line;
    back_bound_line = {left_bound_line.back(), right_bound_line.back()};

    if (
      bg::intersects(footprint_polygon, left_bound_line) ||
      bg::intersects(footprint_polygon, right_bound_line) ||
      bg::intersects(footprint_polygon, back_bound_line)) {
      return true;
    }
    return false;
  }

  const bool front_top_left = isFrontDrivableArea(top_left_pos, left_bound, right_bound);
  const bool front_top_right = isFrontDrivableArea(top_right_pos, left_bound, right_bound);
  const bool front_bottom_left = isFrontDrivableArea(bottom_left_pos, left_bound, right_bound);
  const bool front_bottom_right = isFrontDrivableArea(bottom_right_pos, left_bound, right_bound);

  // create rectangle
  const auto drivable_area_poly = createDrivablePolygon(left_bound, right_bound);

  if (!front_top_left && !bg::within(Point2d(top_left_pos.x, top_left_pos.y), drivable_area_poly)) {
    return true;
  }

  if (
    !front_top_right &&
    !bg::within(Point2d(top_right_pos.x, top_right_pos.y), drivable_area_poly)) {
    return true;
  }

  if (
    !front_bottom_left &&
    !bg::within(Point2d(bottom_left_pos.x, bottom_left_pos.y), drivable_area_poly)) {
    return true;
  }

  if (
    !front_bottom_right &&
    !bg::within(Point2d(bottom_right_pos.x, bottom_right_pos.y), drivable_area_poly)) {
    return true;
  }

  return false;
}
}  // namespace geometry_utils
}  // namespace obstacle_avoidance_planner
