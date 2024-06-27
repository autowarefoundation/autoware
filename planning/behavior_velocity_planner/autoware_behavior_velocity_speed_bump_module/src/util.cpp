// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "autoware/motion_utils/trajectory/path_with_lane_id.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;
using Line = bg::model::linestring<Point>;

using autoware::motion_utils::calcLongitudinalOffsetPoint;
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestSegmentIndex;
using autoware::motion_utils::insertTargetPoint;
using autoware::universe_utils::createPoint;

PathPolygonIntersectionStatus getPathPolygonIntersectionStatus(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos, const size_t max_num)
{
  PathPolygonIntersectionStatus polygon_intersection_status;
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

  auto const & is_first_path_point_inside_polygon = bg::within(first_path_point, polygon);
  auto const & is_last_path_point_inside_polygon = bg::within(last_path_point, polygon);

  // classify first and second intersection points
  if (intersects.empty()) {
    if (is_first_path_point_inside_polygon && is_last_path_point_inside_polygon) {
      polygon_intersection_status.is_path_inside_of_polygon = true;
    } else {
      // do nothing
    }
  } else if (intersects.size() == 1) {
    const auto & p = intersects.at(0);
    if (is_last_path_point_inside_polygon) {
      polygon_intersection_status.first_intersection_point = createPoint(p.x(), p.y(), ego_pos.z);
    } else if (is_first_path_point_inside_polygon) {
      polygon_intersection_status.second_intersection_point = createPoint(p.x(), p.y(), ego_pos.z);
    } else {
      // do nothing
    }
  } else if (intersects.size() == 2) {
    const auto & p0 = intersects.at(0);
    const auto & p1 = intersects.at(1);
    polygon_intersection_status.first_intersection_point = createPoint(p0.x(), p0.y(), ego_pos.z);
    polygon_intersection_status.second_intersection_point = createPoint(p1.x(), p1.y(), ego_pos.z);
  } else {
    // do nothing
  }

  return polygon_intersection_status;
}

bool isNoRelation(const PathPolygonIntersectionStatus & status)
{
  return !status.is_path_inside_of_polygon && !status.first_intersection_point &&
         !status.second_intersection_point;
}

bool insertConstSpeedToPathSection(
  std::vector<PathPointWithLaneId> & output, const size_t start_idx, const size_t end_idx,
  const float speed)
{
  if (start_idx > end_idx) {
    return false;
  }

  const size_t start_idx_clamped = std::min(start_idx, output.size() - 1);
  const size_t end_idx_clamped = std::min(end_idx, output.size() - 1);

  for (size_t i = start_idx_clamped; i <= end_idx_clamped; ++i) {
    auto & p = output.at(i);
    const auto & original_velocity = p.point.longitudinal_velocity_mps;
    p.point.longitudinal_velocity_mps = std::min(original_velocity, speed);
  }
  return true;
}

std::optional<size_t> insertPointWithOffset(
  const geometry_msgs::msg::Point & src_point, const double longitudinal_offset,
  std::vector<PathPointWithLaneId> & output, const double overlap_threshold)
{
  const auto & path = output;
  const auto & target_point = calcLongitudinalOffsetPoint(path, src_point, longitudinal_offset);

  if (!target_point) {
    return {};
  }

  const auto & target_segment_idx = findNearestSegmentIndex(path, *target_point);

  const auto & target_point_idx =
    insertTargetPoint(target_segment_idx, *target_point, output, overlap_threshold);

  return target_point_idx;
}

float calcSlowDownSpeed(const Point32 & p1, const Point32 & p2, const float speed_bump_height)
{
  const float m = (p1.y - p2.y) / (p1.x - p2.x);
  const float b = p1.y - (m * p1.x);

  // y=mx+b
  auto speed = m * speed_bump_height + b;

  return std::clamp(speed, p2.y, p1.y);
}

}  // namespace autoware::behavior_velocity_planner
