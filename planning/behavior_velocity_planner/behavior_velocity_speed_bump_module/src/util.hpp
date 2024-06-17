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

#ifndef UTIL_HPP_
#define UTIL_HPP_

#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/point32.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/primitives/Polygon.h>

namespace autoware::behavior_velocity_planner
{

namespace bg = boost::geometry;

using geometry_msgs::msg::Point32;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

// the status of intersection between path and speed bump
struct PathPolygonIntersectionStatus
{
  bool is_path_inside_of_polygon = false;  // true if path is completely inside the speed bump
                                           // polygon (no intersection point)
  boost::optional<geometry_msgs::msg::Point> first_intersection_point;
  boost::optional<geometry_msgs::msg::Point> second_intersection_point;
};

PathPolygonIntersectionStatus getPathPolygonIntersectionStatus(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos, const size_t max_num);

bool isNoRelation(const PathPolygonIntersectionStatus & status);

bool insertConstSpeedToPathSection(
  std::vector<PathPointWithLaneId> & output, const size_t start_idx, const size_t end_idx,
  const float speed);

std::optional<size_t> insertPointWithOffset(
  const geometry_msgs::msg::Point & src_point, const double longitudinal_offset,
  std::vector<PathPointWithLaneId> & output, const double overlap_threshold = 1e-3);

// returns y (speed) for y=mx+b
float calcSlowDownSpeed(const Point32 & p1, const Point32 & p2, const float speed_bump_height);

}  // namespace autoware::behavior_velocity_planner

#endif  // UTIL_HPP_
