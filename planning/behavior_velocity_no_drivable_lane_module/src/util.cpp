// Copyright 2023 TIER IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "util.hpp"

#include "motion_utils/trajectory/path_with_lane_id.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/marker/virtual_wall_marker_creator.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>

namespace behavior_velocity_planner
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

using motion_utils::calcSignedArcLength;
using tier4_autoware_utils::createPoint;

PathWithNoDrivableLanePolygonIntersection getPathIntersectionWithNoDrivableLanePolygon(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos, const size_t max_num)
{
  PathWithNoDrivableLanePolygonIntersection path_no_drivable_lane_polygon_intersection;
  std::vector<Point> intersects{};

  bool found_max_num = false;
  for (size_t i = 0; i < ego_path.points.size() - 1; ++i) {
    const auto & p_back = ego_path.points.at(i).point.pose.position;
    const auto & p_front = ego_path.points.at(i + 1).point.pose.position;
    const Line segment{{p_back.x, p_back.y}, {p_front.x, p_front.y}};

    std::vector<Point> tmp_intersects{};
    bg::intersection(segment, polygon, tmp_intersects);

    for (const auto & p : tmp_intersects) {
      intersects.push_back(p);
      if (intersects.size() == max_num) {
        found_max_num = true;
        break;
      }
    }

    if (found_max_num) {
      break;
    }
  }

  const auto compare = [&](const Point & p1, const Point & p2) {
    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p1.x(), p1.y(), ego_pos.z));

    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), createPoint(p2.x(), p2.y(), ego_pos.z));

    return dist_l1 < dist_l2;
  };

  std::sort(intersects.begin(), intersects.end(), compare);

  const auto & p_last = ego_path.points.back().point.pose.position;
  const auto & p_first = ego_path.points.front().point.pose.position;
  const Point & last_path_point{p_last.x, p_last.y};
  const Point & first_path_point{p_first.x, p_first.y};

  path_no_drivable_lane_polygon_intersection.is_first_path_point_inside_polygon =
    bg::within(first_path_point, polygon);
  auto const & is_last_path_point_inside_polygon = bg::within(last_path_point, polygon);

  if (
    intersects.empty() &&
    path_no_drivable_lane_polygon_intersection.is_first_path_point_inside_polygon &&
    is_last_path_point_inside_polygon) {
    path_no_drivable_lane_polygon_intersection.is_path_inside_of_polygon = true;
  } else {
    // classify first and second intersection points
    for (size_t i = 0; i < intersects.size(); ++i) {
      const auto & p = intersects.at(i);
      if (
        (intersects.size() == 2 && i == 0) ||
        (intersects.size() == 1 && is_last_path_point_inside_polygon)) {
        path_no_drivable_lane_polygon_intersection.first_intersection_point =
          createPoint(p.x(), p.y(), ego_pos.z);
      } else if (
        (intersects.size() == 2 && i == 1) ||
        (intersects.size() == 1 &&
         path_no_drivable_lane_polygon_intersection.is_first_path_point_inside_polygon)) {
        path_no_drivable_lane_polygon_intersection.second_intersection_point =
          createPoint(p.x(), p.y(), ego_pos.z);
      }
    }
  }
  return path_no_drivable_lane_polygon_intersection;
}
}  // namespace behavior_velocity_planner
