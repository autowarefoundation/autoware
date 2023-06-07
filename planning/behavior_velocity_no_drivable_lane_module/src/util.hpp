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

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_core/primitives/Polygon.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{

namespace bg = boost::geometry;

using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;

// the status of intersection between path and no drivable lane polygon
struct PathWithNoDrivableLanePolygonIntersection
{
  bool is_path_inside_of_polygon = false;  // true if path is completely inside the no drivable lane
                                           // polygon (no intersection point)
  bool is_first_path_point_inside_polygon =
    false;  // true if first path point is inside the no drivable lane polygon
  boost::optional<geometry_msgs::msg::Point> first_intersection_point;
  boost::optional<geometry_msgs::msg::Point> second_intersection_point;
};

PathWithNoDrivableLanePolygonIntersection getPathIntersectionWithNoDrivableLanePolygon(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos, const size_t max_num);

}  // namespace behavior_velocity_planner

#endif  // UTIL_HPP_
