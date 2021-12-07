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
#include <boost/geometry/geometries/segment.hpp>

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
using Point2d = boost::geometry::model::d2::point_xy<double>;
using Segment2d = boost::geometry::model::segment<Point2d>;
using LineString2d = boost::geometry::model::linestring<Point2d>;
using Polygon2d =
  boost::geometry::model::polygon<Point2d, false, false>;  // counter-clockwise, open

template <class T>
Point2d to_bg2d(const T & p)
{
  return Point2d(boost::geometry::get<0>(p), boost::geometry::get<1>(p));
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

inline Polygon2d linestring2polygon(const LineString2d & line_string)
{
  Polygon2d polygon;

  for (const auto & p : line_string) {
    polygon.outer().push_back(p);
  }

  return polygon;
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

  return polygon;
}

inline Polygon2d obj2polygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & shape)
{
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double h = shape.x;
  const double w = shape.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  Polygon2d obj_poly;
  boost::geometry::exterior_ring(obj_poly) = boost::assign::list_of<Point2d>(h / 2.0, w / 2.0)(
    -h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);
  return translate_obj_poly;
}

inline double calcOverlapAreaRate(const Polygon2d & target, const Polygon2d & base)
{
  /* OverlapAreaRate: common area(target && base) / target area */

  if (boost::geometry::within(target, base)) {
    // target is within base, common area = target area
    return 1.0;
  }

  if (!boost::geometry::intersects(target, base)) {
    // target and base has not intersect area
    return 0.0;
  }

  // calculate intersect polygon
  std::vector<Polygon2d> intersects;
  boost::geometry::intersection(target, base, intersects);

  // calculate area of polygon
  double intersect_area = 0.0;
  for (const auto & intersect : intersects) {
    intersect_area += boost::geometry::area(intersect);
  }
  const double target_area = boost::geometry::area(target);
  // specification of boost1.65
  // common area is not intersect area
  const double common_area = target_area - intersect_area;

  return common_area / target_area;
}

inline std::vector<Segment2d> makeSegments(const LineString2d & ls)
{
  std::vector<Segment2d> segments;
  for (size_t i = 0; i < ls.size(); ++i) {
    segments.emplace_back(ls.at(i), ls.at(i + 1));
  }
  return segments;
}

inline geometry_msgs::msg::Polygon toGeomMsg(const Polygon2d & polygon)
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

inline Polygon2d toBoostPoly(const geometry_msgs::msg::Polygon & polygon)
{
  Polygon2d boost_poly;
  for (const auto & point : polygon.points) {
    const Point2d point2d(point.x, point.y);
    boost_poly.outer().push_back(point2d);
  }
  return boost_poly;
}

inline Polygon2d toBoostPoly(const lanelet::BasicPolygon2d & polygon)
{
  Polygon2d boost_poly;
  for (const auto & vec : polygon) {
    const Point2d point2d(vec.x(), vec.y());
    boost_poly.outer().push_back(point2d);
  }

  return boost_poly;
}
}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__BOOST_GEOMETRY_HELPER_HPP_
