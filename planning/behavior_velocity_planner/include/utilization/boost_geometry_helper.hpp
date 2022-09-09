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

#ifndef UTILIZATION__BOOST_GEOMETRY_HELPER_HPP_
#define UTILIZATION__BOOST_GEOMETRY_HELPER_HPP_

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/equals.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <lanelet2_core/primitives/Polygon.h>
#include <tf2/utils.h>

#include <algorithm>
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

inline Polygon2d lines2polygon(const LineString2d & left_line, const LineString2d & right_line)
{
  Polygon2d polygon;

  polygon.outer().push_back(left_line.front());

  for (auto itr = right_line.begin(); itr != right_line.end(); ++itr) {
    polygon.outer().push_back(*itr);
  }

  for (auto itr = left_line.rbegin(); itr != left_line.rend(); ++itr) {
    polygon.outer().push_back(*itr);
  }

  bg::correct(polygon);
  return polygon;
}

inline Polygon2d upScalePolygon(
  const geometry_msgs::msg::Point & position, const Polygon2d & polygon, const double scale)
{
  Polygon2d transformed_polygon;
  // upscale
  for (size_t i = 0; i < polygon.outer().size(); i++) {
    const double upscale_x = (polygon.outer().at(i).x() - position.x) * scale + position.x;
    const double upscale_y = (polygon.outer().at(i).y() - position.y) * scale + position.y;
    transformed_polygon.outer().emplace_back(Point2d(upscale_x, upscale_y));
  }
  return transformed_polygon;
}

inline geometry_msgs::msg::Polygon toGeomPoly(const Polygon2d & polygon)
{
  geometry_msgs::msg::Polygon polygon_msg;
  geometry_msgs::msg::Point32 point_msg;
  for (const auto & p : polygon.outer()) {
    point_msg.x = p.x();
    point_msg.y = p.y();
    polygon_msg.points.push_back(point_msg);
  }
  return polygon_msg;
}
}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__BOOST_GEOMETRY_HELPER_HPP_
