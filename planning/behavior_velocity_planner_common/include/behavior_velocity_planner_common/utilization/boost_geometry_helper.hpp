// Copyright 2020 Tier IV, Inc.
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

#ifndef BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__BOOST_GEOMETRY_HELPER_HPP_
#define BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__BOOST_GEOMETRY_HELPER_HPP_

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <vector>
// cppcheck-suppress unknownMacro
BOOST_GEOMETRY_REGISTER_POINT_3D(geometry_msgs::msg::Point, double, cs::cartesian, x, y, z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  geometry_msgs::msg::Pose, double, cs::cartesian, position.x, position.y, position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  geometry_msgs::msg::PoseWithCovarianceStamped, double, cs::cartesian, pose.pose.position.x,
  pose.pose.position.y, pose.pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_auto_planning_msgs::msg::PathPoint, double, cs::cartesian, pose.position.x,
  pose.position.y, pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_auto_planning_msgs::msg::PathPointWithLaneId, double, cs::cartesian,
  point.pose.position.x, point.pose.position.y, point.pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(
  autoware_auto_planning_msgs::msg::TrajectoryPoint, double, cs::cartesian, pose.position.x,
  pose.position.y, pose.position.z)

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

using Point2d = tier4_autoware_utils::Point2d;
using LineString2d = tier4_autoware_utils::LineString2d;
using Polygon2d = tier4_autoware_utils::Polygon2d;

template <class T>
Point2d to_bg2d(const T & p)
{
  return Point2d(bg::get<0>(p), bg::get<1>(p));
}

template <class T>
LineString2d to_bg2d(const std::vector<T> & vec)
{
  LineString2d ps;
  for (const auto & p : vec) {
    ps.push_back(to_bg2d(p));
  }
  return ps;
}

Polygon2d lines2polygon(const LineString2d & left_line, const LineString2d & right_line);

Polygon2d upScalePolygon(
  const geometry_msgs::msg::Point & position, const Polygon2d & polygon, const double scale);

geometry_msgs::msg::Polygon toGeomPoly(const Polygon2d & polygon);

}  // namespace behavior_velocity_planner

#endif  // BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__BOOST_GEOMETRY_HELPER_HPP_
